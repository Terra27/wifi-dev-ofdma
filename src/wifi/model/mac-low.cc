/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
 * Copyright (c) 2009 MIRKO BANCHI
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *          Mirko Banchi <mk.banchi@gmail.com>
 *          Stefano Avallone <stavallo@unina.it>
 */

#include "ns3/simulator.h"
#include "ns3/log.h"
#include "mac-low.h"
#include "qos-txop.h"
#include "snr-tag.h"
#include "ampdu-tag.h"
#include "wifi-mac-queue.h"
#include "wifi-psdu.h"
#include "wifi-utils.h"
#include "ctrl-headers.h"
#include "mgt-headers.h"
#include "wifi-remote-station-manager.h"
#include "mpdu-aggregator.h"
#include "msdu-aggregator.h"
#include "ampdu-subframe-header.h"
#include "wifi-phy-listener.h"
#include "wifi-mac-trailer.h"
#include "wifi-phy.h"
#include "wifi-net-device.h"
#include "ap-wifi-mac.h"
#include "sta-wifi-mac.h"
#include <algorithm>
#include "wifi-ack-policy-selector.h"
#include "ofdma-manager.h"
#include "he-configuration.h"

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT std::clog << "[mac=" << m_self << "] "

// Time (in nanoseconds) to be added to the PSDU duration to yield the duration
// of the timer that is started when the PHY indicates the start of the reception
// of a frame and we are waiting for a response.
#define PSDU_DURATION_SAFEGUARD 400

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MacLow");

/**
 * Listener for PHY events. Forwards to MacLow
 */
class PhyMacLowListener : public ns3::WifiPhyListener
{
public:
  /**
   * Create a PhyMacLowListener for the given MacLow.
   *
   * \param macLow
   */
  PhyMacLowListener (ns3::MacLow *macLow)
    : m_macLow (macLow)
  {
  }
  virtual ~PhyMacLowListener ()
  {
  }
  void NotifyRxStart (Time duration)
  {
  }
  void NotifyRxEndOk (void)
  {
  }
  void NotifyRxEndError (void)
  {
  }
  void NotifyTxStart (Time duration, double txPowerDbm)
  {
  }
  void NotifyMaybeCcaBusyStart (Time duration)
  {
  }
  void NotifySwitchingStart (Time duration)
  {
    m_macLow->NotifySwitchingStartNow (duration);
  }
  void NotifySleep (void)
  {
    m_macLow->NotifySleepNow ();
  }
  void NotifyOff (void)
  {
    m_macLow->NotifyOffNow ();
  }
  void NotifyWakeup (void)
  {
  }
  void NotifyOn (void)
  {
  }

private:
  ns3::MacLow *m_macLow; ///< the MAC
};


MacLow::MacLow ()
  : m_msduAggregator (0),
    m_mpduAggregator (0),
    m_normalAckTimeoutEvent (),
    m_blockAckTimeoutEvent (),
    m_ctsTimeoutEvent (),
    m_sendCtsEvent (),
    m_sendAckEvent (),
    m_sendDataEvent (),
    m_waitIfsEvent (),
    m_endTxNoAckEvent (),
    m_currentTxop (0),
    m_lastNavStart (Seconds (0)),
    m_lastNavDuration (Seconds (0)),
    m_cfpStart (Seconds (0)),
    m_lastBeacon (Seconds (0)),
    m_cfpForeshortening (Seconds (0)),
    m_promisc (false),
    m_phyMacLowListener (0),
    m_ctsToSelfSupported (false),
    m_cfAckInfo ()
{
  NS_LOG_FUNCTION (this);
}

MacLow::~MacLow ()
{
  NS_LOG_FUNCTION (this);
}

/* static */
TypeId
MacLow::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MacLow")
    .SetParent<Object> ()
    .SetGroupName ("Wifi")
    .AddConstructor<MacLow> ()
    .AddTraceSource ("ForwardDown",
                     "The PSDU map forwarded down to the PHY along with the TX vector.",
                     MakeTraceSourceAccessor (&MacLow::m_forwardDown),
                     "ns3::WifiTxVector::TracedCallback")
  ;
  return tid;
}

void
MacLow::SetupPhyMacLowListener (const Ptr<WifiPhy> phy)
{
  m_phyMacLowListener = new PhyMacLowListener (this);
  phy->RegisterListener (m_phyMacLowListener);
}

void
MacLow::RemovePhyMacLowListener (Ptr<WifiPhy> phy)
{
  if (m_phyMacLowListener != 0 )
    {
      phy->UnregisterListener (m_phyMacLowListener);
      delete m_phyMacLowListener;
      m_phyMacLowListener = 0;
    }
}

void
MacLow::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_normalAckTimeoutEvent.Cancel ();
  m_blockAckTimeoutEvent.Cancel ();
  m_ctsTimeoutEvent.Cancel ();
  m_sendCtsEvent.Cancel ();
  m_sendAckEvent.Cancel ();
  m_sendDataEvent.Cancel ();
  m_waitIfsEvent.Cancel ();
  m_endTxNoAckEvent.Cancel ();
  m_msduAggregator = 0;
  m_mpduAggregator = 0;
  m_ofdmaManager = 0;
  m_phy = 0;
  m_stationManager = 0;
  if (m_phyMacLowListener != 0)
    {
      delete m_phyMacLowListener;
      m_phyMacLowListener = 0;
    }
  m_currentPacket.clear ();
}

void
MacLow::CancelAllEvents (void)
{
  NS_LOG_FUNCTION (this);
  bool oneRunning = false;
  if (m_normalAckTimeoutEvent.IsRunning ())
    {
      m_normalAckTimeoutEvent.Cancel ();
      oneRunning = true;
    }
  if (m_blockAckTimeoutEvent.IsRunning ())
    {
      m_blockAckTimeoutEvent.Cancel ();
      oneRunning = true;
    }
  if (m_ctsTimeoutEvent.IsRunning ())
    {
      m_ctsTimeoutEvent.Cancel ();
      oneRunning = true;
    }
  if (m_sendCtsEvent.IsRunning ())
    {
      m_sendCtsEvent.Cancel ();
      oneRunning = true;
    }
  if (m_sendAckEvent.IsRunning ())
    {
      m_sendAckEvent.Cancel ();
      oneRunning = true;
    }
  if (m_sendDataEvent.IsRunning ())
    {
      m_sendDataEvent.Cancel ();
      oneRunning = true;
    }
  if (m_waitIfsEvent.IsRunning ())
    {
      m_waitIfsEvent.Cancel ();
      oneRunning = true;
    }
  if (m_endTxNoAckEvent.IsRunning ())
    {
      m_endTxNoAckEvent.Cancel ();
      oneRunning = true;
    }
  if (oneRunning && m_currentTxop != 0)
    {
      m_currentTxop->Cancel ();
      m_currentTxop = 0;
    }
}

void
MacLow::SetPhy (const Ptr<WifiPhy> phy)
{
  m_phy = phy;
  m_phy->TraceConnectWithoutContext ("PhyRxPayloadBegin", MakeCallback (&MacLow::RxStartIndication, this));
  m_phy->SetReceiveOkCallback (MakeCallback (&MacLow::DeaggregateAmpduAndReceive, this));
  m_phy->SetReceiveErrorCallback (MakeCallback (&MacLow::ReceiveError, this));
  SetupPhyMacLowListener (phy);
}

Ptr<WifiPhy>
MacLow::GetPhy (void) const
{
  return m_phy;
}

void
MacLow::ResetPhy (void)
{
  m_phy->TraceDisconnectWithoutContext ("PhyRxPayloadBegin", MakeCallback (&MacLow::RxStartIndication, this));
  m_phy->SetReceiveOkCallback (MakeNullCallback<void, Ptr<WifiPsdu>, RxSignalInfo, WifiTxVector, std::vector<bool>> ());
  m_phy->SetReceiveErrorCallback (MakeNullCallback<void, Ptr<WifiPsdu>> ());
  RemovePhyMacLowListener (m_phy);
  m_phy = 0;
}

Ptr<QosTxop>
MacLow::GetEdca (uint8_t tid) const
{
  auto it = m_edca.find (QosUtilsMapTidToAc (tid));
  NS_ASSERT (it != m_edca.end ());
  return it->second;
}

void
MacLow::SetMac (const Ptr<WifiMac> mac)
{
  m_mac = mac;
}

void
MacLow::SetOfdmaManager (const Ptr<OfdmaManager> ofdmaManager)
{
  NS_ASSERT (m_mac);
  Ptr<ApWifiMac> apMac = DynamicCast<ApWifiMac> (m_mac);
  NS_ABORT_MSG_IF (apMac == 0, "APs only can be aggregated to OFDMA Managers");
  NS_ABORT_MSG_IF (m_mac->GetHeConfiguration () == 0, "HE APs only can be aggregated to OFDMA Managers");
  m_ofdmaManager = ofdmaManager;
}

void
MacLow::SetWifiRemoteStationManager (const Ptr<WifiRemoteStationManager> manager)
{
  m_stationManager = manager;
}

Ptr<MsduAggregator>
MacLow::GetMsduAggregator (void) const
{
  return m_msduAggregator;
}

Ptr<MpduAggregator>
MacLow::GetMpduAggregator (void) const
{
  return m_mpduAggregator;
}

void
MacLow::SetMsduAggregator (const Ptr<MsduAggregator> aggr)
{
  NS_LOG_FUNCTION (this << aggr);
  m_msduAggregator = aggr;
}

void
MacLow::SetMpduAggregator (const Ptr<MpduAggregator> aggr)
{
  NS_LOG_FUNCTION (this << aggr);
  m_mpduAggregator = aggr;
}

void
MacLow::SetAddress (Mac48Address ad)
{
  m_self = ad;
}

void
MacLow::SetAckTimeout (Time ackTimeout)
{
  m_ackTimeout = ackTimeout;
}

void
MacLow::SetBasicBlockAckTimeout (Time blockAckTimeout)
{
  m_basicBlockAckTimeout = blockAckTimeout;
}

void
MacLow::SetCompressedBlockAckTimeout (Time blockAckTimeout)
{
  m_compressedBlockAckTimeout = blockAckTimeout;
}

void
MacLow::SetCtsToSelfSupported (bool enable)
{
  m_ctsToSelfSupported = enable;
}

bool
MacLow::GetCtsToSelfSupported (void) const
{
  return m_ctsToSelfSupported;
}

void
MacLow::SetCtsTimeout (Time ctsTimeout)
{
  m_ctsTimeout = ctsTimeout;
}

void
MacLow::SetSifs (Time sifs)
{
  m_sifs = sifs;
}

void
MacLow::SetSlotTime (Time slotTime)
{
  m_slotTime = slotTime;
}

void
MacLow::SetPifs (Time pifs)
{
  m_pifs = pifs;
}

void
MacLow::SetRifs (Time rifs)
{
  m_rifs = rifs;
}

void
MacLow::SetBeaconInterval (Time interval)
{
  m_beaconInterval = interval;
}

void
MacLow::SetCfpMaxDuration (Time cfpMaxDuration)
{
  m_cfpMaxDuration = cfpMaxDuration;
}

void
MacLow::SetBssid (Mac48Address bssid)
{
  m_bssid = bssid;
}

void
MacLow::SetPromisc (void)
{
  m_promisc = true;
}

void
MacLow::SetContinueTxopIfNoSuResponseAfterMuPpdu (bool continueTxop)
{
  m_continueTxopIfNoSuResponseAfterMuPpdu = continueTxop;
}

bool
MacLow::GetContinueTxopIfNoSuResponseAfterMuPpdu () const
{
  return m_continueTxopIfNoSuResponseAfterMuPpdu;
}

Mac48Address
MacLow::GetAddress (void) const
{
  return m_self;
}

Time
MacLow::GetAckTimeout (void) const
{
  return m_ackTimeout;
}

Time
MacLow::GetBasicBlockAckTimeout (void) const
{
  return m_basicBlockAckTimeout;
}

Time
MacLow::GetCompressedBlockAckTimeout (void) const
{
  return m_compressedBlockAckTimeout;
}

Time
MacLow::GetCtsTimeout (void) const
{
  return m_ctsTimeout;
}

Time
MacLow::GetSifs (void) const
{
  return m_sifs;
}

Time
MacLow::GetRifs (void) const
{
  return m_rifs;
}

Time
MacLow::GetSlotTime (void) const
{
  return m_slotTime;
}

Time
MacLow::GetPifs (void) const
{
  return m_pifs;
}

Mac48Address
MacLow::GetBssid (void) const
{
  return m_bssid;
}

Time
MacLow::GetBeaconInterval (void) const
{
  return m_beaconInterval;
}

Time
MacLow::GetCfpMaxDuration (void) const
{
  return m_cfpMaxDuration;
}

bool
MacLow::IsPromisc (void) const
{
  return m_promisc;
}

void
MacLow::SetRxCallback (Callback<void, Ptr<WifiMacQueueItem>> callback)
{
  m_rxCallback = callback;
}

void
MacLow::RegisterDcf (Ptr<ChannelAccessManager> dcf)
{
  m_channelAccessManagers.push_back (dcf);
}

void
MacLow::StartTransmission (Ptr<WifiMacQueueItem> mpdu,
                           MacLowTransmissionParameters params,
                           Ptr<Txop> txop)
{
  NS_LOG_FUNCTION (this << *mpdu << params << txop);
  NS_ASSERT (!m_cfAckInfo.expectCfAck);
  if (m_phy->IsStateOff ())
    {
      NS_LOG_DEBUG ("Cannot start TX because device is OFF");
      return;
    }
  /* m_currentPacket is not NULL because someone started
   * a transmission and was interrupted before one of:
   *   - ctsTimeout
   *   - sendDataAfterCTS
   * expired. This means that one of these timers is still
   * running. They are all cancelled below anyway by the
   * call to CancelAllEvents (because of at least one
   * of these two timers) which will trigger a call to the
   * previous listener's cancel method.
   *
   * This typically happens because the high-priority
   * QapScheduler has taken access to the channel from
   * one of the Edca of the QAP.
   */
  const WifiMacHeader& hdr = mpdu->GetHeader ();
  CancelAllEvents ();
  m_currentTxop = txop;
  m_txParams = params;
  if (hdr.IsCtl ())
    {
      m_currentTxVector = GetRtsTxVector (mpdu);
    }
  else
    {
      m_currentTxVector = GetDataTxVector (mpdu);
    }

  /* The packet received by this function can be any of the following:
   * (a) a management frame dequeued from the Txop
   * (b) a non-QoS data frame dequeued from the Txop
   * (c) a non-broadcast QoS Data frame peeked or dequeued from a QosTxop
   * (d) a broadcast QoS data or DELBA Request frame dequeued from a QosTxop
   * (e) a BlockAckReq or ADDBA Request frame
   * (f) a fragment of non-QoS/QoS Data frame dequeued from the Txop/QosTxop
   */

  // A VHT/HE frame is transmitted as S-MPDU (if not aggregated)
  WifiModulationClass modulation = m_currentTxVector.GetMode ().GetModulationClass ();
  bool smpdu = (modulation == WIFI_MOD_CLASS_VHT || modulation == WIFI_MOD_CLASS_HE);
  m_currentPacket.clear ();
  m_currentPacket[SU_STA_ID] = Create<WifiPsdu> (mpdu, smpdu);

  if (hdr.IsQosData () && !hdr.GetAddr1 ().IsBroadcast ()
      && !hdr.IsMoreFragments () && hdr.GetFragmentNumber () == 0)
    {
      m_currentPacket.clear ();
      bool useReceivedMpdu = false;
      // check if QosTxop sent us a peeked frame
      Ptr<QosTxop> currentQosTxop = DynamicCast<QosTxop> (m_currentTxop);
      NS_ASSERT (currentQosTxop != 0);
      auto peekedMpdu = currentQosTxop->PeekNextFrame ();
      bool isReceivedMpduPeeked = (peekedMpdu != 0 && peekedMpdu->GetPacket () == mpdu->GetPacket ());

      // We get here if the received packet is a non-broadcast QoS data frame.
      // If this is an HE AP with an OfdmaManager installed, check if the next
      // transmission needs to be DL OFDMA, UL OFDMA or non-OFDMA
      OfdmaTxFormat txFormat = OfdmaTxFormat::NON_OFDMA;

      if (m_ofdmaManager != 0)
        {
          m_ofdmaManager->NotifyAccessGranted (mpdu);
          txFormat = m_ofdmaManager->GetTxFormat ();
        }

      if (txFormat == OfdmaTxFormat::NON_OFDMA
          || (txFormat == OfdmaTxFormat::DL_OFDMA && !m_ofdmaManager->GetDlOfdmaInfo ().staInfo.empty ()))
        {
          /* Downlink transmission */
          std::map<Mac48Address,OfdmaManager::DlPerStaInfo>::const_iterator dlOfdmaInfoIt;
          if (txFormat == OfdmaTxFormat::DL_OFDMA)
            {
              NS_LOG_DEBUG ("Starting a DL OFDMA transmission");
              dlOfdmaInfoIt = m_ofdmaManager->GetDlOfdmaInfo ().staInfo.begin ();
              m_currentTxVector = m_ofdmaManager->GetDlOfdmaInfo ().txVector;
              m_txParams = m_ofdmaManager->GetDlOfdmaInfo ().params;
            }

          // if a TXOP limit exists, compute the remaining TXOP duration
          Time ppduDurationLimit = Seconds (0);
          if (m_currentTxop->GetTxopLimit ().IsStrictlyPositive ())
            {
              ppduDurationLimit = m_currentTxop->GetTxopRemaining () - CalculateOverheadTxTime (mpdu, m_txParams, m_currentTxVector);
              if (ppduDurationLimit.IsNegative ())
                {
                  NS_LOG_DEBUG (" Return because the remaining TXOP time is less than the response duration");
                  return;
                }
            }

          // The following loop handles the preparation of both SU PPDUs (in
          // which case a single iteration is performed) and MU PPDUs.
          do
            {
              Ptr<WifiMacQueueItem> newMpdu;
              Mac48Address rcv = hdr.GetAddr1 ();
              uint8_t tid = hdr.GetQosTid ();
              uint16_t aid = SU_STA_ID;
              uint32_t ampduSize = 0;
              uint32_t muBarSize = 0;

              if (txFormat == OfdmaTxFormat::DL_OFDMA)
                {
                  // determine the next receiver-TID pair
                  rcv = dlOfdmaInfoIt->first;
                  tid = dlOfdmaInfoIt->second.tid;
                  aid = dlOfdmaInfoIt->second.aid;

                  if (m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF)
                    {
                      NS_ASSERT (m_mpduAggregator != 0);

                      // we need to take into account that MU-BAR TFs will be aggregated to each A-MPDU
                      muBarSize = GetMuBarSize ({GetEdca (tid)->GetBlockAckReqType (rcv, tid)});
                      ampduSize = MpduAggregator::GetSizeIfAggregated (muBarSize, 0);
                    }

                  NS_LOG_DEBUG ("The next PSDU has to be sent to " << rcv << ", tid=" << +tid);
                  NS_ABORT_MSG_IF (QosUtilsMapTidToAc (tid) < QosUtilsMapTidToAc (hdr.GetQosTid ()),
                                   "Only PSDUs of the AC that gained access to the channel"
                                   << "or higher can be added to an MU PPDU (Got: "
                                   << QosUtilsMapTidToAc (tid) << " Expected at least: "
                                   << QosUtilsMapTidToAc (hdr.GetQosTid ()) << ")");
                }

              Ptr<QosTxop> qosTxop = GetEdca (tid);

              if (rcv == hdr.GetAddr1 () && tid == hdr.GetQosTid ())
                {
                  // If the mpdu passed by the caller has been peeked, dequeue it if
                  // it meets the size and duration constraints
                  if (isReceivedMpduPeeked)
                    {
                      newMpdu = qosTxop->DequeuePeekedFrame (mpdu, m_currentTxVector, true, ampduSize, ppduDurationLimit);
                    }
                  // otherwise, use it if it meets the constraints
                  else if (IsWithinSizeAndTimeLimits (mpdu, m_currentTxVector, ampduSize, ppduDurationLimit))
                    {
                      newMpdu = mpdu;
                    }

                  if (newMpdu != 0)
                    {
                      useReceivedMpdu = true;
                      qosTxop->UpdateCurrentPacket (newMpdu);
                    }
                }
              else
                {
                  Ptr<const WifiMacQueueItem> peeked;
                  peeked = qosTxop->PeekNextFrame (tid, rcv);

                  if (peeked)
                    {
                      // null is returned if the peeked frame does not meet size and time constraints
                      newMpdu = qosTxop->DequeuePeekedFrame (peeked, m_currentTxVector, true, ampduSize, ppduDurationLimit);
                    }
                  else
                    {
                      NS_LOG_DEBUG ("No available frame to be sent to " << rcv << ", tid=" << +tid);
                    }
                }

              if (newMpdu)
                {
                  std::vector<Ptr<WifiMacQueueItem>> mpduList;

                  //Perform A-MPDU aggregation if possible
                  if (m_mpduAggregator != 0)
                    {
                      mpduList = m_mpduAggregator->GetNextAmpdu (newMpdu, m_currentTxVector, ppduDurationLimit, muBarSize);
                    }

                  if (mpduList.size () > 1)
                    {
                      m_currentPacket[aid] = Create<WifiPsdu> (mpduList);

                      NS_LOG_DEBUG ("tx unicast A-MPDU containing " << mpduList.size () << " MPDUs");
                      qosTxop->SetAmpduExist (rcv, true);
                    }
                  else if (m_currentTxVector.GetMode (aid).GetModulationClass () == WIFI_MOD_CLASS_VHT
                           || m_currentTxVector.GetMode (aid).GetModulationClass () == WIFI_MOD_CLASS_HE)
                    {
                      // VHT/HE single MPDU
                      m_currentPacket[aid] = Create<WifiPsdu> (newMpdu, true);

                      NS_LOG_DEBUG ("tx unicast S-MPDU with sequence number " << hdr.GetSequenceNumber ());
                      qosTxop->SetAmpduExist (hdr.GetAddr1 (), true);
                    }
                  else   // HT
                    {
                      m_currentPacket[aid] = Create<WifiPsdu> (newMpdu, false);
                    }
                }
            } while (txFormat == OfdmaTxFormat::DL_OFDMA &&
                     ++dlOfdmaInfoIt != m_ofdmaManager->GetDlOfdmaInfo ().staInfo.end ());

          // A QoS Txop must have an installed ack policy selector
          NS_ASSERT (currentQosTxop->GetAckPolicySelector () != 0);
          currentQosTxop->GetAckPolicySelector ()->UpdateTxParams (m_currentPacket, m_txParams);
          WifiAckPolicySelector::SetAckPolicy (m_currentPacket, m_txParams);

          // Aggregate MU-BAR Trigger Frames if needed
          if (m_txParams.HasDlMuAckSequence () &&
              m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF)
            {
              CtrlTriggerHeader trigger = m_ofdmaManager->GetDlOfdmaInfo ().trigger;
              trigger.SetUlLength (CalculateUlLengthForBlockAcks (trigger, m_txParams));

              for (auto& psdu : m_currentPacket)
                {
                  Mac48Address rcv = psdu.second->GetAddr1 ();
                  uint8_t tid = *psdu.second->GetTids ().begin ();
                  auto muBar = GetEdca (tid)->PrepareMuBar (trigger, {{psdu.first, {rcv, tid}}});

                  std::vector<Ptr<WifiMacQueueItem>> mpduList (psdu.second->begin (), psdu.second->end ());
                  mpduList.push_back (Copy (muBar));
                  m_currentPacket[psdu.first] = Create<WifiPsdu> (mpduList);
                }
            }
        }
      if (!useReceivedMpdu && !isReceivedMpduPeeked)
        {
          // the MPDU received from the caller is not being transmitted. If the MPDU
          // was just peeked by the QosTxop, there is nothing to do. If the MPDU was
          // dequeued by the QosTxop, we need to re-insert it in the appropriate queue.
          // Since QosTxop only peeks MPDUs when a Block Ack agreement is established
          // with the receiver, the received MPDU must be inserted in the EDCA queue.
          NS_LOG_DEBUG ("Storing received MPDU (" << *mpdu << ") in the EDCA queue");
          currentQosTxop->GetWifiMacQueue ()->PushFront (mpdu);
        }
    }

  if (m_currentPacket.empty ())
    {
      NS_LOG_DEBUG ("No frame to transmit");
      // reset current packet, terminate TXOP and restart access
      m_currentTxop->EndTxNoAck ();
      return;
    }

  for (auto& psdu : m_currentPacket)
    {
      NS_LOG_DEBUG ("startTx size=" << psdu.second->GetSize () << ", to="
                    << psdu.second->GetAddr1 () << ", txop=" << m_currentTxop);
    }

  if (m_txParams.MustSendRts ())
    {
      SendRtsForPacket ();
    }
  else
    {
      if ((m_ctsToSelfSupported || m_stationManager->GetUseNonErpProtection ()) && NeedCtsToSelf ())
        {
          SendCtsToSelf ();
        }
      else
        {
          SendDataPacket ();
        }
    }

  /* When this method completes, either we have taken ownership of the medium or the device switched off in the meantime. */
  NS_ASSERT (m_phy->IsStateTx () || m_phy->IsStateOff ());
}

bool
MacLow::NeedCtsToSelf (void) const
{
  if (m_currentPacket.size () > 1)
    {
      // CTS-to-self before an MU PPDU is not supported yet
      return false;
    }

  WifiTxVector dataTxVector = GetDataTxVector (*m_currentPacket.at (SU_STA_ID)->begin ());
  return m_stationManager->NeedCtsToSelf (dataTxVector);
}

bool
MacLow::IsWithinSizeAndTimeLimits (Ptr<const WifiMacQueueItem> mpdu, WifiTxVector txVector,
                                    uint32_t ampduSize, Time ppduDurationLimit)
{
  NS_ASSERT (mpdu != 0 && mpdu->GetHeader ().IsQosData ());

  return IsWithinSizeAndTimeLimits (mpdu->GetSize (), mpdu->GetHeader ().GetAddr1 (),
                                    mpdu->GetHeader ().GetQosTid (), txVector,
                                    ampduSize, ppduDurationLimit);
}

bool
MacLow::IsWithinSizeAndTimeLimits (uint32_t mpduSize, Mac48Address receiver, uint8_t tid,
                                    WifiTxVector txVector, uint32_t ampduSize, Time ppduDurationLimit)
{
  NS_LOG_FUNCTION (this << mpduSize << receiver << +tid << txVector << ampduSize << ppduDurationLimit);

  uint16_t staId = GetStaId (receiver);
  WifiModulationClass modulation = txVector.GetMode (staId).GetModulationClass ();

  uint32_t maxAmpduSize = 0;
  if (GetMpduAggregator ())
    {
      maxAmpduSize = GetMpduAggregator ()->GetMaxAmpduSize (receiver, tid, modulation);
    }

  // If maxAmpduSize is null, then ampduSize must be null as well
  NS_ASSERT (maxAmpduSize || ampduSize == 0);

  uint32_t ppduPayloadSize = mpduSize;

  // compute the correct size for A-MPDUs and S-MPDUs
  if (ampduSize > 0 || modulation == WIFI_MOD_CLASS_HE || modulation == WIFI_MOD_CLASS_VHT)
    {
      ppduPayloadSize = GetMpduAggregator ()->GetSizeIfAggregated (mpduSize, ampduSize);
    }

  NS_LOG_DEBUG ("A-MPDU size: " << ppduPayloadSize);

  if (maxAmpduSize > 0 && ppduPayloadSize > maxAmpduSize)
    {
      NS_LOG_DEBUG ("the frame does not meet the constraint on max A-MPDU size ("
                    << maxAmpduSize << ")");
      return false;
    }

  // Get the maximum PPDU Duration based on the preamble type
  Time maxPpduDuration = GetPpduMaxTime (txVector.GetPreambleType ());

  Time txTime = m_phy->CalculateTxDuration (ppduPayloadSize, txVector, m_phy->GetFrequency (), staId);
  NS_LOG_DEBUG ("PPDU duration: " << txTime.ToDouble (Time::MS) << "ms");

  if ((ppduDurationLimit.IsStrictlyPositive () && txTime > ppduDurationLimit)
      || (maxPpduDuration.IsStrictlyPositive () && txTime > maxPpduDuration))
    {
      NS_LOG_DEBUG ("the frame does not meet the constraint on max PPDU duration");
      return false;
    }

  return true;
}

void
MacLow::RxStartIndication (WifiTxVector txVector, Time psduDuration)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("PSDU reception started for " << psduDuration.ToDouble (Time::US)
                << " us (txVector: " << txVector << ")");
  NS_ASSERT (psduDuration.IsStrictlyPositive ());

  if (m_normalAckTimeoutEvent.IsRunning ())
    {
      // we are waiting for a Normal Ack and something arrived
      NS_LOG_DEBUG ("Rescheduling Normal Ack timeout");
      m_normalAckTimeoutEvent.Cancel ();
      NotifyAckTimeoutResetNow ();
      m_normalAckTimeoutEvent = Simulator::Schedule (psduDuration + NanoSeconds (PSDU_DURATION_SAFEGUARD),
                                                     &MacLow::NormalAckTimeout, this);
    }
  else if (m_blockAckTimeoutEvent.IsRunning ())
    {
      // we are waiting for a Normal Ack and something arrived
      NS_LOG_DEBUG ("Rescheduling Block Ack timeout");
      m_blockAckTimeoutEvent.Cancel ();
      NotifyAckTimeoutResetNow ();
      m_blockAckTimeoutEvent = Simulator::Schedule (psduDuration + NanoSeconds (PSDU_DURATION_SAFEGUARD),
                                                     &MacLow::BlockAckTimeout, this);
    }

  return;
}

void
MacLow::ReceiveError (Ptr<WifiPsdu> psdu)
{
  NS_LOG_FUNCTION (this << *psdu);
  NS_LOG_DEBUG ("rx failed");
  if (IsCfPeriod () && m_currentPacket.at (SU_STA_ID)->GetHeader (0).IsCfPoll ())
    {
      NS_ASSERT (m_currentTxop != 0);
      m_currentTxop->MissedCfPollResponse (m_cfAckInfo.expectCfAck);
    }
  else if (m_cfAckInfo.expectCfAck)
    {
      NS_ASSERT (m_currentTxop != 0);
      Ptr<Txop> txop = m_currentTxop;
      m_currentTxop = 0;
      txop->MissedAck ();
    }
  m_cfAckInfo.expectCfAck = false;
  return;
}

void
MacLow::NotifySwitchingStartNow (Time duration)
{
  NS_LOG_DEBUG ("switching channel. Cancelling MAC pending events");
  m_stationManager->Reset ();
  CancelAllEvents ();
  if (m_navCounterResetCtsMissed.IsRunning ())
    {
      m_navCounterResetCtsMissed.Cancel ();
    }
  m_lastNavStart = Simulator::Now ();
  m_lastNavDuration = Seconds (0);
  m_currentPacket.clear ();
  m_currentTxop = 0;
}

void
MacLow::NotifySleepNow (void)
{
  NS_LOG_DEBUG ("Device in sleep mode. Cancelling MAC pending events");
  CancelAllEvents ();
  if (m_navCounterResetCtsMissed.IsRunning ())
    {
      m_navCounterResetCtsMissed.Cancel ();
    }
  m_lastNavStart = Simulator::Now ();
  m_lastNavDuration = Seconds (0);
  m_currentPacket.clear ();
  m_currentTxop = 0;
}

void
MacLow::NotifyOffNow (void)
{
  NS_LOG_DEBUG ("Device is switched off. Cancelling MAC pending events");
  CancelAllEvents ();
  if (m_navCounterResetCtsMissed.IsRunning ())
    {
      m_navCounterResetCtsMissed.Cancel ();
    }
  m_lastNavStart = Simulator::Now ();
  m_lastNavDuration = Seconds (0);
  m_currentPacket.clear ();
  m_currentTxop = 0;
}

void
MacLow::ReceiveOk (Ptr<WifiMacQueueItem> mpdu, RxSignalInfo rxSignalInfo, WifiTxVector txVector, bool ampduSubframe)
{
  NS_LOG_FUNCTION (this << *mpdu << rxSignalInfo << txVector);
  /* An MPDU is received from the PHY.
   * When we have handled this MPDU,
   * we handle any packet present in the
   * packet queue.
   */
  const WifiMacHeader& hdr = mpdu->GetHeader ();
  Ptr<Packet> packet = mpdu->GetPacket ()->Copy ();
  uint16_t staId = GetStaId (m_self);

  CtrlTriggerHeader trigger;
  if (hdr.IsTrigger ())
    {
      packet->RemoveHeader (trigger);
    }

  bool isPrevNavZero = IsNavZero ();
  NS_LOG_DEBUG ("duration/id=" << hdr.GetDuration ());
  NotifyNav (packet, hdr);
  double rxSnr = rxSignalInfo.snr;
  if (hdr.IsRts ())
    {
      /* see section 9.2.5.7 802.11-1999
       * A STA that is addressed by an RTS frame shall transmit a CTS frame after a SIFS
       * period if the NAV at the STA receiving the RTS frame indicates that the medium is
       * idle. If the NAV at the STA receiving the RTS indicates the medium is not idle,
       * that STA shall not respond to the RTS frame.
       */
      if (ampduSubframe)
        {
          NS_FATAL_ERROR ("Received RTS as part of an A-MPDU");
        }
      else
        {
          if (isPrevNavZero
              && hdr.GetAddr1 () == m_self)
            {
              NS_LOG_DEBUG ("rx RTS from=" << hdr.GetAddr2 () << ", schedule CTS");
              NS_ASSERT (m_sendCtsEvent.IsExpired ());
              m_stationManager->ReportRxOk (hdr.GetAddr2 (), &hdr,
                                            rxSignalInfo, txVector.GetMode ());
              m_sendCtsEvent = Simulator::Schedule (GetSifs (),
                                                    &MacLow::SendCtsAfterRts, this,
                                                    hdr.GetAddr2 (),
                                                    hdr.GetDuration (),
                                                    txVector,
                                                    rxSnr);
            }
          else
            {
              NS_LOG_DEBUG ("rx RTS from=" << hdr.GetAddr2 () << ", cannot schedule CTS");
            }
        }
    }
  else if (hdr.IsCts ()
           && hdr.GetAddr1 () == m_self
           && m_ctsTimeoutEvent.IsRunning ()
           && !m_currentPacket.empty ())
    {
      NS_ABORT_MSG_IF (ampduSubframe, "Received CTS as part of an A-MPDU");
      NS_ABORT_MSG_IF (m_currentPacket.size () > 1, "Received CTS when the current packet is an MU PPDU");

      Ptr<WifiPsdu> psdu = m_currentPacket.begin ()->second;
      NS_LOG_DEBUG ("received cts from=" << psdu->GetAddr1 ());

      SnrTag tag;
      packet->RemovePacketTag (tag);
      m_stationManager->ReportRxOk (psdu->GetAddr1 (), &psdu->GetHeader (0),
                                    rxSignalInfo, txVector.GetMode ());
      m_stationManager->ReportRtsOk (psdu->GetAddr1 (), &psdu->GetHeader (0),
                                     rxSnr, txVector.GetMode (), tag.Get ());

      m_ctsTimeoutEvent.Cancel ();
      NotifyCtsTimeoutResetNow ();
      NS_ASSERT (m_sendDataEvent.IsExpired ());
      m_sendDataEvent = Simulator::Schedule (GetSifs (),
                                             &MacLow::SendDataAfterCts, this,
                                             hdr.GetDuration ());
    }
  else if (hdr.IsAck ()
           && hdr.GetAddr1 () == m_self
           && m_normalAckTimeoutEvent.IsRunning ()
           && (m_txParams.MustWaitNormalAck () || !m_txParams.GetStationsReceiveAckFrom ().empty ()))
    {
      Ptr<WifiPsdu> psdu;
      if (!m_txParams.GetStationsReceiveAckFrom ().empty ())
        {
          // received ack for an S-MPDU transmitted within a DL MU PPDU
          auto psduIt = m_currentPacket.find (GetStaId (m_txParams.GetStationsReceiveAckFrom ().front ()));
          NS_ASSERT (psduIt != m_currentPacket.end ());
          psdu = psduIt->second;
        }
      else
        {
          NS_ASSERT (m_currentPacket.size () == 1);
          psdu = m_currentPacket.begin ()->second;
        }
      NS_LOG_DEBUG ("receive ack from=" << psdu->GetAddr1 ());
      SnrTag tag;
      packet->RemovePacketTag (tag);
      //When fragmentation is used, only update manager when the last fragment is acknowledged
      if (!m_txParams.HasNextPacket ())
        {
          m_stationManager->ReportRxOk (psdu->GetAddr1 (), &psdu->GetHeader (0),
                                        rxSignalInfo, txVector.GetMode ());
          m_stationManager->ReportDataOk (psdu->GetAddr1 (), &psdu->GetHeader (0),
                                          rxSnr, txVector.GetMode (), tag.Get (),
                                          psdu->GetSize ());
        }
      // cancel the Normal Ack timer
      m_normalAckTimeoutEvent.Cancel ();
      NotifyAckTimeoutResetNow ();
      // notify the Txop
      if (m_txParams.MustWaitNormalAck ())
        {
          m_currentTxop->GotAck ();
        }
      else
        {
          // in case of S-MPDU included in MU DL PPDU, we need to notify the AC the S-MPDU
          // belongs to (which may be different than the AC that transmitted the MU DL PPDU)
          NS_ASSERT (psdu->GetNMpdus () == 1 && psdu->GetHeader (0).IsQosData ());
          Ptr<QosTxop> qosTxop = m_edca[QosUtilsMapTidToAc (psdu->GetHeader (0).GetQosTid ())];
          qosTxop->UpdateCurrentPacket (*psdu->begin ());
          qosTxop->GotAck ();
        }

      if (m_txParams.HasNextPacket ())
        {
          if (m_stationManager->GetRifsPermitted ())
            {
              m_waitIfsEvent = Simulator::Schedule (GetRifs (), &MacLow::WaitIfsAfterEndTxFragment, this);
            }
          else
            {
              m_waitIfsEvent = Simulator::Schedule (GetSifs (), &MacLow::WaitIfsAfterEndTxFragment, this);
            }
        }
      else if (m_currentTxop->GetTxopLimit ().IsStrictlyPositive () && m_currentTxop->GetTxopRemaining () > GetSifs ())
        {
          if (m_stationManager->GetRifsPermitted ())
            {
              m_waitIfsEvent = Simulator::Schedule (GetRifs (), &MacLow::WaitIfsAfterEndTxPacket, this);
            }
          else
            {
              m_waitIfsEvent = Simulator::Schedule (GetSifs (), &MacLow::WaitIfsAfterEndTxPacket, this);
            }
        }
      else if (m_currentTxop->IsQosTxop ())
        {
          m_currentTxop->TerminateTxop ();
        }
    }
  else if (hdr.IsBlockAck () && hdr.GetAddr1 () == m_self
           && (m_txParams.MustWaitBlockAck () || !m_txParams.GetStationsReceiveBlockAckFrom ().empty ())
           && m_blockAckTimeoutEvent.IsRunning ())
    {
      NS_LOG_DEBUG ("got block ack from " << hdr.GetAddr2 ());
      SnrTag tag;
      packet->RemovePacketTag (tag);
      CtrlBAckResponseHeader blockAck;
      packet->RemoveHeader (blockAck);
      // notify the QosTxop about the reception of the block ack
      Ptr<QosTxop> qosTxop = m_edca.find (QosUtilsMapTidToAc (blockAck.GetTidInfo ()))->second;
      qosTxop->GotBlockAck (&blockAck, hdr.GetAddr2 (), rxSnr,
                            txVector.GetMode (GetStaId (hdr.GetAddr2 ())), tag.Get ());
      // if this block ack was sent in response to an MU PPDU, remove the station that
      // sent the block ack from the TX params list
      std::list<Mac48Address> staList = m_txParams.GetStationsReceiveBlockAckFrom ();
      if (!staList.empty ())
        {
          NS_ASSERT (std::find (staList.begin (), staList.end (), hdr.GetAddr2 ()) != staList.end ());
          // remove the sender of the Block Ack from the list of stations which
          // we expect a Block Ack from
          m_txParams.DisableAck (hdr.GetAddr2 ());
        }
      // if we do not expect any other Block Ack, cancel the timer and schedule
      // the transmission of the next frame in the TXOP, if any
      if (m_txParams.MustWaitBlockAck () || m_txParams.GetStationsReceiveBlockAckFrom ().empty ())
        {
          m_blockAckTimeoutEvent.Cancel ();
          NotifyAckTimeoutResetNow ();

          // start next packet if TXOP remains, otherwise contend for accessing the channel again
          if (m_currentTxop->IsQosTxop () && m_currentTxop->GetTxopLimit ().IsStrictlyPositive ()
              && m_currentTxop->GetTxopRemaining () > GetSifs ())
            {
              if (m_stationManager->GetRifsPermitted ())
                {
                  m_waitIfsEvent = Simulator::Schedule (GetRifs (), &MacLow::WaitIfsAfterEndTxPacket, this);
                }
              else
                {
                  m_waitIfsEvent = Simulator::Schedule (GetSifs (), &MacLow::WaitIfsAfterEndTxPacket, this);
                }
            }
          else if (m_currentTxop->IsQosTxop ())
            {
              m_currentTxop->TerminateTxop ();
            }
        }
    }
  else if ((hdr.IsBlockAckReq () && hdr.GetAddr1 () == m_self)
           || (hdr.IsTrigger () && trigger.IsMuBar ()
               && (hdr.GetAddr1 () == m_self
                   || (hdr.GetAddr1 ().IsBroadcast ()
                       && trigger.FindUserInfoWithAid (GetStaId (m_self)) != trigger.end ()))))
    {
      CtrlBAckRequestHeader blockAckReq;
      WifiTxVector blockAckTxVector;
      if (hdr.IsBlockAckReq ())
        {
          packet->RemoveHeader (blockAckReq);
          blockAckTxVector = GetBlockAckTxVector (hdr.GetAddr2 (), txVector.GetMode ());
        }
      else
        {
          auto userInfoIt = trigger.FindUserInfoWithAid (GetStaId (m_self));
          NS_ASSERT (userInfoIt != trigger.end ());
          blockAckReq = userInfoIt->GetMuBarTriggerDepUserInfo ();
          blockAckTxVector = trigger.GetHeTbTxVector (GetStaId (m_self));
        }

      if (!blockAckReq.IsMultiTid ())
        {
          uint8_t tid = blockAckReq.GetTidInfo ();
          AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), tid));
          if (it != m_bAckAgreements.end ())
            {
              //Update block ack cache
              BlockAckCachesI i = m_bAckCaches.find (std::make_pair (hdr.GetAddr2 (), tid));
              NS_ASSERT (i != m_bAckCaches.end ());
              (*i).second.UpdateWithBlockAckReq (blockAckReq.GetStartingSequence ());

              //NS_ASSERT (m_sendAckEvent.IsExpired ());
              m_sendAckEvent.Cancel ();
              /* See section 11.5.3 in IEEE 802.11 for mean of this timer */
              ResetBlockAckInactivityTimerIfNeeded (it->second.first);
              if ((*it).second.first.IsImmediateBlockAck ())
                {
                  NS_LOG_DEBUG ("rx blockAckRequest/sendImmediateBlockAck from=" << hdr.GetAddr2 ());
                  m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                                        &MacLow::SendBlockAckAfterBlockAckRequest, this,
                                                        blockAckReq,
                                                        hdr.GetAddr2 (),
                                                        hdr.GetDuration (),
                                                        blockAckTxVector,
                                                        rxSnr);
                }
              else
                {
                  NS_FATAL_ERROR ("Delayed block ack not supported.");
                }
            }
          else
            {
              NS_LOG_DEBUG ("There's not a valid agreement for this block ack request.");
            }
        }
      else
        {
          NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
        }
    }
  else if (hdr.IsCtl ())
    {
      if (hdr.IsCfEnd ())
        {
          NS_LOG_DEBUG ("rx CF-END ");
          m_cfpStart = NanoSeconds (0);
          if (m_cfAckInfo.expectCfAck)
            {
              NS_ASSERT (m_currentTxop != 0);
              if (hdr.IsCfAck ())
                {
                  m_currentTxop->GotAck ();
                }
              else
                {
                  m_currentTxop->MissedAck ();
                }
            }
          if (m_currentTxop != 0)
            {
              m_currentTxop->GotCfEnd ();
            }
          m_cfAckInfo.expectCfAck = false;
        }
      else
        {
          NS_LOG_DEBUG ("rx drop " << hdr.GetTypeString ());
        }
    }
  else if (hdr.GetAddr1 () == m_self)
    {
      if (hdr.IsCfPoll ())
        {
          m_cfpStart = Simulator::Now ();
          if (m_cfAckInfo.expectCfAck && !hdr.IsCfAck ())
            {
              NS_ASSERT (m_currentTxop != 0);
              Ptr<Txop> txop = m_currentTxop;
              m_currentTxop = 0;
              txop->MissedAck ();
              m_cfAckInfo.expectCfAck = false;
            }
        }
      m_stationManager->ReportRxOk (hdr.GetAddr2 (), &hdr,
                                    rxSignalInfo, txVector.GetMode (staId));
      if (hdr.IsQosData () && ReceiveMpdu (mpdu))
        {
          /* From section 9.10.4 in IEEE 802.11:
             Upon the receipt of a QoS data frame from the originator for which
             the Block Ack agreement exists, the recipient shall buffer the MSDU
             regardless of the value of the Ack Policy subfield within the
             QoS Control field of the QoS data frame. */
          if (hdr.IsQosAck () && !ampduSubframe)
            {
              NS_LOG_DEBUG ("rx QoS unicast/sendAck from=" << hdr.GetAddr2 ());
              AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));

              RxCompleteBufferedPacketsWithSmallerSequence (it->second.first.GetStartingSequenceControl (),
                                                            hdr.GetAddr2 (), hdr.GetQosTid ());
              RxCompleteBufferedPacketsUntilFirstLost (hdr.GetAddr2 (), hdr.GetQosTid ());
              NS_ASSERT (m_sendAckEvent.IsExpired ());
              m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                                    &MacLow::SendAckAfterData, this,
                                                    hdr.GetAddr2 (),
                                                    hdr.GetDuration (),
                                                    txVector.GetMode (staId),
                                                    rxSnr);
            }
          else if (hdr.IsQosBlockAck ())
            {
              AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
              /* See section 11.5.3 in IEEE 802.11 for mean of this timer */
              ResetBlockAckInactivityTimerIfNeeded (it->second.first);
            }
          return;
        }
      else if (hdr.IsQosData () && hdr.IsQosBlockAck ())
        {
          /* This happens if a packet with ack policy Block Ack is received and a block ack
             agreement for that packet doesn't exist.

             From section 11.5.3 in IEEE 802.11e:
             When a recipient does not have an active Block ack for a TID, but receives
             data MPDUs with the Ack Policy subfield set to Block Ack, it shall discard
             them and shall send a DELBA frame using the normal access
             mechanisms. */
          AcIndex ac = QosUtilsMapTidToAc (hdr.GetQosTid ());
          m_edca[ac]->SendDelbaFrame (hdr.GetAddr2 (), hdr.GetQosTid (), false);
          return;
        }
      else if (hdr.IsQosData () && hdr.IsQosNoAck ())
        {
          if (ampduSubframe)
            {
              NS_LOG_DEBUG ("rx Ampdu with No Ack Policy from=" << hdr.GetAddr2 ());
            }
          else
            {
              NS_LOG_DEBUG ("rx unicast/noAck from=" << hdr.GetAddr2 ());
            }
        }
      else if (hdr.IsData () || hdr.IsMgt ())
        {
          if (hdr.IsProbeResp ())
            {
              // Apply SNR tag for probe response quality measurements
              SnrTag tag;
              tag.Set (rxSnr);
              packet->AddPacketTag (tag);
            }
          if (hdr.IsMgt () && ampduSubframe)
            {
              NS_FATAL_ERROR ("Received management packet as part of an A-MPDU");
            }
          else
            {
              if (IsCfPeriod ())
                {
                  if (hdr.HasData ())
                    {
                      m_cfAckInfo.appendCfAck = true;
                      m_cfAckInfo.address = hdr.GetAddr2 ();
                    }
                }
              else
                {
                  NS_LOG_DEBUG ("rx unicast/sendAck from=" << hdr.GetAddr2 ());
                  NS_ASSERT (m_sendAckEvent.IsExpired ());
                  m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                                        &MacLow::SendAckAfterData, this,
                                                        hdr.GetAddr2 (),
                                                        hdr.GetDuration (),
                                                        txVector.GetMode (),
                                                        rxSnr);
                }
            }
        }
      goto rxPacket;
    }
  else if (hdr.GetAddr1 ().IsGroup ())
    {
      if (ampduSubframe)
        {
          NS_FATAL_ERROR ("Received group addressed packet as part of an A-MPDU");
        }
      else
        {
          if (hdr.IsData () || hdr.IsMgt ())
            {
              NS_LOG_DEBUG ("rx group from=" << hdr.GetAddr2 ());
              if (hdr.IsBeacon ())
                {
                  // Apply SNR tag for beacon quality measurements
                  SnrTag tag;
                  tag.Set (rxSnr);
                  packet->AddPacketTag (tag);
                }
              goto rxPacket;
            }
        }
    }
  else if (m_promisc)
    {
      NS_ASSERT (hdr.GetAddr1 () != m_self);
      if (hdr.IsData ())
        {
          goto rxPacket;
        }
    }
  else
    {
      if (m_cfAckInfo.expectCfAck && hdr.IsCfAck ())
        {
          m_cfAckInfo.expectCfAck = false;
          NS_ASSERT (m_currentTxop != 0);
          m_currentTxop->GotAck ();
        }
      NS_LOG_DEBUG ("rx not for me from=" << hdr.GetAddr2 ());
    }
  return;
rxPacket:
  if (m_cfAckInfo.expectCfAck && hdr.IsCfAck ())
    {
      m_cfAckInfo.expectCfAck = false;
      NS_ASSERT (m_currentTxop != 0);
      m_currentTxop->GotAck ();
    }
  if (hdr.IsQosData ())
    {
      m_rxCallback (mpdu);
    }
  else
    {
      // We cannot pass mpdu to the callback here because a tag is attached to some
      // management frames (beacons and probe responses) by this method. If we passed
      // mpdu to m_rxCallback here, given that management frames are received by
      // multiple stations, a tag of the same type would be attached multiple times,
      // thus crashing simulations. We avoid the problem by creating a new packet at
      // every receiver.
      m_rxCallback (Create<WifiMacQueueItem> (packet, hdr));
    }
  return;
}

uint32_t
MacLow::GetCfEndSize (void) const
{
  WifiMacHeader cfEnd;
  if (m_cfAckInfo.expectCfAck || m_cfAckInfo.appendCfAck)
    {
      cfEnd.SetType (WIFI_MAC_CTL_END_ACK);
    }
  else
    {
      cfEnd.SetType (WIFI_MAC_CTL_END);
    }
  return cfEnd.GetSize () + 4;
}

Time
MacLow::GetAckDuration (Mac48Address to, WifiTxVector dataTxVector) const
{
  WifiTxVector ackTxVector = GetAckTxVectorForData (to, dataTxVector.GetMode ());
  return GetAckDuration (ackTxVector);
}

Time
MacLow::GetAckDuration (WifiTxVector ackTxVector) const
{
  NS_ASSERT (ackTxVector.GetMode ().GetModulationClass () != WIFI_MOD_CLASS_HT); //ACK should always use non-HT PPDU (HT PPDU cases not supported yet)
  return m_phy->CalculateTxDuration (GetAckSize (), ackTxVector, m_phy->GetFrequency ());
}

Time
MacLow::GetBlockAckDuration (WifiTxVector blockAckReqTxVector, BlockAckType type, uint16_t staId) const
{
  /*
   * For immediate Basic BlockAck we should transmit the frame with the same WifiMode
   * as the BlockAckReq.
   */
  return m_phy->CalculateTxDuration (GetBlockAckSize (type), blockAckReqTxVector, m_phy->GetFrequency (), staId);
}

Time
MacLow::GetBlockAckRequestDuration (WifiTxVector blockAckReqTxVector, BlockAckReqType type) const
{
  return m_phy->CalculateTxDuration (GetBlockAckRequestSize (type), blockAckReqTxVector, m_phy->GetFrequency ());
}

Time
MacLow::GetCtsDuration (Mac48Address to, WifiTxVector rtsTxVector) const
{
  WifiTxVector ctsTxVector = GetCtsTxVectorForRts (to, rtsTxVector.GetMode ());
  return GetCtsDuration (ctsTxVector);
}

Time
MacLow::GetCtsDuration (WifiTxVector ctsTxVector) const
{
  NS_ASSERT (ctsTxVector.GetMode ().GetModulationClass () != WIFI_MOD_CLASS_HT); //CTS should always use non-HT PPDU (HT PPDU cases not supported yet)
  return m_phy->CalculateTxDuration (GetCtsSize (), ctsTxVector, m_phy->GetFrequency ());
}

WifiTxVector
MacLow::GetRtsTxVector (Ptr<const WifiMacQueueItem> item) const
{
  Mac48Address to = item->GetHeader ().GetAddr1 ();
  return m_stationManager->GetRtsTxVector (to, &item->GetHeader (), item->GetPacket ());
}

WifiTxVector
MacLow::GetDataTxVector (Ptr<const WifiMacQueueItem> item) const
{
  Mac48Address to = item->GetHeader ().GetAddr1 ();
  return m_stationManager->GetDataTxVector (to, &item->GetHeader (), item->GetPacket ());
}

uint16_t
MacLow::CalculateUlLengthForBlockAcks (CtrlTriggerHeader trigger, const MacLowTransmissionParameters& params) const
{
  NS_LOG_FUNCTION (this << trigger << params);
  NS_ASSERT (params.HasDlMuAckSequence ());
  NS_ASSERT (params.GetDlMuAckSequenceType () != DlMuAckSequenceType::DL_SU_FORMAT);

  std::list<Mac48Address> staList;

  if (params.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_MU_BAR)
    {
      staList = params.GetStationsSendBlockAckRequestTo ();
    }
  else
    {
      staList = params.GetStationsReceiveBlockAckFrom ();
    }

  NS_ASSERT (!staList.empty ());
  Time maxBlockAckDuration = Seconds (0);

  for (auto& sta : staList)
    {
      // compute the TX duration of the Block Ack response from this receiver.
      Time currBlockAckDuration = m_phy->CalculateTxDuration (GetBlockAckSize (params.GetBlockAckType (sta)),
                                                              trigger.GetHeTbTxVector (GetStaId (sta)),
                                                              m_phy->GetFrequency (),
                                                              GetStaId (sta));
      // update the max duration among all the Block Ack responses
      if (currBlockAckDuration > maxBlockAckDuration)
        {
          maxBlockAckDuration = currBlockAckDuration;
        }
    }

  NS_LOG_DEBUG ("TX duration of HE TB PPDU containing Block Acks: " << maxBlockAckDuration.ToDouble (Time::US) << " us");
  return WifiPhy::ConvertHeTbPpduDurationToLSigLength (maxBlockAckDuration, m_phy->GetFrequency ());
}

Time
MacLow::GetResponseDuration (const MacLowTransmissionParameters& params, WifiTxVector txVector,
                             Ptr<const WifiMacQueueItem> mpdu) const
{
  NS_LOG_FUNCTION (this << params << txVector << mpdu);

  Time duration = Seconds (0);
  if (params.MustWaitNormalAck ())
    {
      duration += GetSifs ();
      duration += GetAckDuration (mpdu->GetHeader ().GetAddr1 (), txVector);
    }
  else if (params.MustWaitBlockAck ())
    {
      duration += GetSifs ();
      WifiTxVector blockAckReqTxVector = GetBlockAckTxVector (m_self, txVector.GetMode ());
      duration += GetBlockAckDuration (blockAckReqTxVector, params.GetBlockAckType ());
    }
  else if (params.MustSendBlockAckRequest ())
    {
      duration += 2 * GetSifs ();
      WifiTxVector blockAckReqTxVector = GetBlockAckTxVector (m_self, txVector.GetMode ());
      duration += GetBlockAckRequestDuration (blockAckReqTxVector, params.GetBlockAckRequestType ());
      duration += GetBlockAckDuration (blockAckReqTxVector, params.GetBlockAckType ());
    }
  else if (mpdu != 0 && mpdu->GetHeader ().IsTrigger ())
    {
      CtrlTriggerHeader trigger;
      mpdu->GetPacket ()->PeekHeader (trigger);

      /** MU-BAR Trigger Frame soliciting Block Acks in a HE TB PPDU **/
      if (trigger.IsMuBar ())
        {
          // SIFS + Block Acks (as an HE TB PPDU)
          duration += GetSifs ();
          duration += WifiPhy::ConvertLSigLengthToHeTbPpduDuration (trigger.GetUlLength (),
                                                                    trigger.GetHeTbTxVector (trigger.begin ()->GetAid12 ()),
                                                                    m_phy->GetFrequency ());
        }
    }
  /*
   * DL MU PPDU
   */
  else if (params.HasDlMuAckSequence ())
    {
      auto txVectorUserInfo = txVector.GetHeMuUserInfoMap ();

      std::list<Mac48Address> staReceiveAckFrom = params.GetStationsReceiveAckFrom (),
                              staReceiveBlockAckFrom = params.GetStationsReceiveBlockAckFrom (),
                              staSendBlockAckRequestTo = params.GetStationsSendBlockAckRequestTo ();

      NS_ASSERT (m_mac);
      Ptr<ApWifiMac> apMac = DynamicCast<ApWifiMac> (m_mac);
      NS_ASSERT_MSG (apMac != 0, "APs only can transmit DL MU PPDUs");
      auto aidAddressMap = apMac->GetStaList ();

      /** DL MU PPDU - Acknowledgment via a sequence of BAR/BA pairs **/
      if (params.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_SU_FORMAT)
        {
          // normal ack or implicit BAR policy can be used for (no more than) one receiver
          NS_ASSERT (staReceiveAckFrom.size () + staReceiveBlockAckFrom.size () <= 1);

          // iterate over the STA_ID_LIST of the TX vector
          for (auto& userInfo : txVectorUserInfo)
            {
              // search the STA-ID in the map of associated stations
              auto addressIt = aidAddressMap.find (userInfo.first);
              NS_ASSERT (addressIt != aidAddressMap.end ());

              std::list<Mac48Address>::iterator it;
              // check if this station has to send a normal ack
              if ((it = std::find (staReceiveAckFrom.begin (), staReceiveAckFrom.end (), addressIt->second))
                  != staReceiveAckFrom.end ())
                {
                  duration += GetSifs ();
                  WifiTxVector ackTxVector = GetAckTxVectorForData (addressIt->second, userInfo.second.mcs);
                  duration += GetAckDuration (ackTxVector);
                  // remove the station from the list, so we can check that all stations have been handled
                  staReceiveAckFrom.erase (it);
                }
              // check if this station has to send a block ack
              else if ((it = std::find (staReceiveBlockAckFrom.begin (), staReceiveBlockAckFrom.end (), addressIt->second))
                       != staReceiveBlockAckFrom.end ())
                {
                  duration += GetSifs ();
                  WifiTxVector blockAckReqTxVector = GetBlockAckTxVector (m_self, userInfo.second.mcs);
                  duration += GetBlockAckDuration (blockAckReqTxVector, params.GetBlockAckType (addressIt->second));
                  // remove the station from the list, so we can check that all stations have been handled
                  staReceiveBlockAckFrom.erase (it);
                }
              // check if this station has to receive a block ack request and reply with a block ack
              else if ((it = std::find (staSendBlockAckRequestTo.begin (), staSendBlockAckRequestTo.end (), addressIt->second))
                       != staSendBlockAckRequestTo.end ())
                {
                  duration += 2 * GetSifs ();
                  WifiTxVector blockAckReqTxVector = GetBlockAckTxVector (m_self, userInfo.second.mcs);
                  duration += GetBlockAckRequestDuration (blockAckReqTxVector, params.GetBlockAckRequestType (addressIt->second));
                  duration += GetBlockAckDuration (blockAckReqTxVector, params.GetBlockAckType (addressIt->second));
                  // remove the station from the list, so we can check that all stations have been handled
                  staSendBlockAckRequestTo.erase (it);
                }
            }
          // all the stations in the TX params must have been handled
          NS_ASSERT (staReceiveAckFrom.empty () && staReceiveBlockAckFrom.empty () && staSendBlockAckRequestTo.empty ());
        }
      /** DL MU PPDU - Acknowledgment via a MU-BAR and Block Acks in a HE TB PPDU **/
      else if (params.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_MU_BAR)
        {
          // no station has to send a normal ack or a block ack
          NS_ASSERT (staReceiveAckFrom.empty () && staReceiveBlockAckFrom.empty ());
          // we can only get here if this is an HE AP with an aggregated OFDMA Manager
          NS_ASSERT (m_ofdmaManager != 0);

          std::list<BlockAckReqType> barTypes;
          for (auto& sta : staSendBlockAckRequestTo)
            {
              barTypes.push_back (params.GetBlockAckRequestType (sta));
            }
          WifiMacHeader hdr;
          hdr.SetType (WIFI_MAC_CTL_TRIGGER);
          hdr.SetAddr1 (Mac48Address::GetBroadcast ());
          uint32_t muBarSize = GetMuBarSize (barTypes);

          // SIFS + MU-BAR + SIFS + Block Acks (as an HE TB PPDU)
          duration += 2 * GetSifs ();
          duration += m_phy->CalculateTxDuration (muBarSize,
                                                  GetRtsTxVector (Create<WifiMacQueueItem> (Create<Packet> (muBarSize), hdr)),
                                                  m_phy->GetFrequency ());

          const CtrlTriggerHeader& trigger = m_ofdmaManager->GetDlOfdmaInfo ().trigger;
          duration += WifiPhy::ConvertLSigLengthToHeTbPpduDuration (trigger.GetUlLength (),
                                                                    trigger.GetHeTbTxVector (trigger.begin ()->GetAid12 ()),
                                                                    m_phy->GetFrequency ());
        }
      /** DL MU PPDU - Acknowledgment via aggregated MU-BARs and Block Acks in a HE TB PPDU **/
      else if (params.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF)
        {
          // no station has to send a normal ack or receive a block ack request
          NS_ASSERT (staReceiveAckFrom.empty () && staSendBlockAckRequestTo.empty ());
          // we can only get here if this is an HE AP with an aggregated OFDMA Manager
          NS_ASSERT (m_ofdmaManager != 0);

          // SIFS + Block Acks (as an HE TB PPDU)
          duration += GetSifs ();

          const CtrlTriggerHeader& trigger = m_ofdmaManager->GetDlOfdmaInfo ().trigger;
          duration += WifiPhy::ConvertLSigLengthToHeTbPpduDuration (trigger.GetUlLength (),
                                                                    trigger.GetHeTbTxVector (trigger.begin ()->GetAid12 ()),
                                                                    m_phy->GetFrequency ());
        }
    }

  NS_LOG_DEBUG ("Response duration: " << duration.ToDouble (Time::MS) << "ms");
  return duration;
}

WifiMode
MacLow::GetControlAnswerMode (WifiMode reqMode) const
{
  /**
   * The standard has relatively unambiguous rules for selecting a
   * control response rate (the below is quoted from IEEE 802.11-2012,
   * Section 9.7):
   *
   * To allow the transmitting STA to calculate the contents of the
   * Duration/ID field, a STA responding to a received frame shall
   * transmit its Control Response frame (either CTS or ACK), other
   * than the BlockAck control frame, at the highest rate in the
   * BSSBasicRateSet parameter that is less than or equal to the
   * rate of the immediately previous frame in the frame exchange
   * sequence (as defined in Annex G) and that is of the same
   * modulation class (see Section 9.7.8) as the received frame...
   */
  NS_LOG_FUNCTION (this << reqMode);
  WifiMode mode = m_stationManager->GetDefaultMode ();
  bool found = false;
  //First, search the BSS Basic Rate set
  for (uint8_t i = 0; i < m_stationManager->GetNBasicModes (); i++)
    {
      WifiMode testMode = m_stationManager->GetBasicMode (i);
      if ((!found || testMode.IsHigherDataRate (mode))
          && (!testMode.IsHigherDataRate (reqMode))
          && (IsAllowedControlAnswerModulationClass (reqMode.GetModulationClass (), testMode.GetModulationClass ())))
        {
          mode = testMode;
          //We've found a potentially-suitable transmit rate, but we
          //need to continue and consider all the basic rates before
          //we can be sure we've got the right one.
          found = true;
        }
    }
  if (m_stationManager->GetHtSupported () || m_stationManager->GetVhtSupported () || m_stationManager->GetHeSupported ())
    {
      if (!found)
        {
          mode = m_stationManager->GetDefaultMcs ();
          for (uint8_t i = 0; i != m_stationManager->GetNBasicMcs (); i++)
            {
              WifiMode testMode = m_stationManager->GetBasicMcs (i);
              if ((!found || testMode.IsHigherDataRate (mode))
                  && (!testMode.IsHigherDataRate (reqMode))
                  && (testMode.GetModulationClass () == reqMode.GetModulationClass ()))
                {
                  mode = testMode;
                  //We've found a potentially-suitable transmit rate, but we
                  //need to continue and consider all the basic rates before
                  //we can be sure we've got the right one.
                  found = true;
                }
            }
        }
    }
  //If we found a suitable rate in the BSSBasicRateSet, then we are
  //done and can return that mode.
  if (found)
    {
      NS_LOG_DEBUG ("MacLow::GetControlAnswerMode returning " << mode);
      return mode;
    }

  /**
   * If no suitable basic rate was found, we search the mandatory
   * rates. The standard (IEEE 802.11-2007, Section 9.6) says:
   *
   *   ...If no rate contained in the BSSBasicRateSet parameter meets
   *   these conditions, then the control frame sent in response to a
   *   received frame shall be transmitted at the highest mandatory
   *   rate of the PHY that is less than or equal to the rate of the
   *   received frame, and that is of the same modulation class as the
   *   received frame. In addition, the Control Response frame shall
   *   be sent using the same PHY options as the received frame,
   *   unless they conflict with the requirement to use the
   *   BSSBasicRateSet parameter.
   *
   * \todo Note that we're ignoring the last sentence for now, because
   * there is not yet any manipulation here of PHY options.
   */
  for (uint8_t idx = 0; idx < m_phy->GetNModes (); idx++)
    {
      WifiMode thismode = m_phy->GetMode (idx);
      /* If the rate:
       *
       *  - is a mandatory rate for the PHY, and
       *  - is equal to or faster than our current best choice, and
       *  - is less than or equal to the rate of the received frame, and
       *  - is of the same modulation class as the received frame
       *
       * ...then it's our best choice so far.
       */
      if (thismode.IsMandatory ()
          && (!found || thismode.IsHigherDataRate (mode))
          && (!thismode.IsHigherDataRate (reqMode))
          && (IsAllowedControlAnswerModulationClass (reqMode.GetModulationClass (), thismode.GetModulationClass ())))
        {
          mode = thismode;
          //As above; we've found a potentially-suitable transmit
          //rate, but we need to continue and consider all the
          //mandatory rates before we can be sure we've got the right one.
          found = true;
        }
    }
  if (m_stationManager->GetHtSupported () || m_stationManager->GetVhtSupported () || m_stationManager->GetHeSupported ())
    {
      for (uint8_t idx = 0; idx < m_phy->GetNMcs (); idx++)
        {
          WifiMode thismode = m_phy->GetMcs (idx);
          if (thismode.IsMandatory ()
              && (!found || thismode.IsHigherDataRate (mode))
              && (!thismode.IsHigherCodeRate (reqMode))
              && (thismode.GetModulationClass () == reqMode.GetModulationClass ()))
            {
              mode = thismode;
              //As above; we've found a potentially-suitable transmit
              //rate, but we need to continue and consider all the
              //mandatory rates before we can be sure we've got the right one.
              found = true;
            }
        }
    }

  /**
   * If we still haven't found a suitable rate for the response then
   * someone has messed up the simulation config. This probably means
   * that the WifiPhyStandard is not set correctly, or that a rate that
   * is not supported by the PHY has been explicitly requested.
   *
   * Either way, it is serious - we can either disobey the standard or
   * fail, and I have chosen to do the latter...
   */
  if (!found)
    {
      NS_FATAL_ERROR ("Can't find response rate for " << reqMode);
    }

  NS_LOG_DEBUG ("MacLow::GetControlAnswerMode returning " << mode);
  return mode;
}

WifiTxVector
MacLow::GetCtsTxVector (Mac48Address to, WifiMode rtsTxMode) const
{
  NS_ASSERT (!to.IsGroup ());
  WifiMode ctsMode = GetControlAnswerMode (rtsTxMode);
  WifiTxVector v;
  v.SetMode (ctsMode);
  v.SetPreambleType (GetPreambleForTransmission (ctsMode.GetModulationClass (), m_stationManager->GetShortPreambleEnabled (), m_stationManager->UseGreenfieldForDestination (to)));
  v.SetTxPowerLevel (m_stationManager->GetDefaultTxPowerLevel ());
  v.SetChannelWidth (GetChannelWidthForTransmission (ctsMode, m_phy->GetChannelWidth ()));
   uint16_t ctsTxGuardInterval = ConvertGuardIntervalToNanoSeconds (ctsMode, m_phy->GetShortGuardInterval (), m_phy->GetGuardInterval ());
  v.SetGuardInterval (ctsTxGuardInterval);
  v.SetNss (1);
  return v;
}

WifiTxVector
MacLow::GetAckTxVector (Mac48Address to, WifiMode dataTxMode) const
{
  NS_ASSERT (!to.IsGroup ());
  WifiMode ackMode = GetControlAnswerMode (dataTxMode);
  WifiTxVector v;
  v.SetMode (ackMode);
  v.SetPreambleType (GetPreambleForTransmission (ackMode.GetModulationClass (), m_stationManager->GetShortPreambleEnabled (), m_stationManager->UseGreenfieldForDestination (to)));
  v.SetTxPowerLevel (m_stationManager->GetDefaultTxPowerLevel ());
  v.SetChannelWidth (GetChannelWidthForTransmission (ackMode, m_phy->GetChannelWidth ()));
   uint16_t ackTxGuardInterval = ConvertGuardIntervalToNanoSeconds (ackMode, m_phy->GetShortGuardInterval (), m_phy->GetGuardInterval ());
  v.SetGuardInterval (ackTxGuardInterval);
  v.SetNss (1);
  return v;
}

WifiTxVector
MacLow::GetBlockAckTxVector (Mac48Address to, WifiMode dataTxMode) const
{
  NS_ASSERT (!to.IsGroup ());
  WifiMode blockAckMode = GetControlAnswerMode (dataTxMode);
  WifiTxVector v;
  v.SetMode (blockAckMode);
  v.SetPreambleType (GetPreambleForTransmission (blockAckMode.GetModulationClass (), m_stationManager->GetShortPreambleEnabled (), m_stationManager->UseGreenfieldForDestination (to)));
  v.SetTxPowerLevel (m_stationManager->GetDefaultTxPowerLevel ());
  v.SetChannelWidth (GetChannelWidthForTransmission (blockAckMode, m_phy->GetChannelWidth ()));
uint16_t blockAckTxGuardInterval = ConvertGuardIntervalToNanoSeconds (blockAckMode, m_phy->GetShortGuardInterval (), m_phy->GetGuardInterval ());
  v.SetGuardInterval (blockAckTxGuardInterval);
  v.SetNss (1);
  return v;
}

WifiTxVector
MacLow::GetCtsTxVectorForRts (Mac48Address to, WifiMode rtsTxMode) const
{
  return GetCtsTxVector (to, rtsTxMode);
}

WifiTxVector
MacLow::GetAckTxVectorForData (Mac48Address to, WifiMode dataTxMode) const
{
  return GetAckTxVector (to, dataTxMode);
}

Time
MacLow::CalculateOverallTxTime (Ptr<const Packet> packet,
                                const WifiMacHeader* hdr,
                                const MacLowTransmissionParameters& params,
                                uint32_t fragmentSize) const
{
  Ptr<const WifiMacQueueItem> item = Create<const WifiMacQueueItem> (packet, *hdr);
  Time txTime = CalculateOverheadTxTime (item, params);
  uint32_t dataSize;
  if (fragmentSize > 0)
    {
      Ptr<const Packet> fragment = Create<Packet> (fragmentSize);
      dataSize = GetSize (fragment, hdr, false);
    }
  else
    {
      dataSize = GetSize (packet, hdr, false);
    }
  txTime += m_phy->CalculateTxDuration (dataSize, GetDataTxVector (item), m_phy->GetFrequency ());
  return txTime;
}

Time
MacLow::CalculateOverheadTxTime (Ptr<const WifiMacQueueItem> item,
                                 const MacLowTransmissionParameters& params) const
{
  WifiTxVector txVector = (item->GetHeader ().IsCtl () ? GetRtsTxVector (item) : GetDataTxVector (item));
  return CalculateOverheadTxTime (item, params, txVector);
}

Time
MacLow::CalculateOverheadTxTime (Ptr<const WifiMacQueueItem> item,
                                 const MacLowTransmissionParameters& params, WifiTxVector dataTxVector) const
{
  Time txTime = Seconds (0);
  if (params.MustSendRts ())
    {
      WifiTxVector rtsTxVector = GetRtsTxVector (item);
      txTime += m_phy->CalculateTxDuration (GetRtsSize (), rtsTxVector, m_phy->GetFrequency ());
      txTime += GetCtsDuration (item->GetHeader ().GetAddr1 (), rtsTxVector);
      txTime += Time (GetSifs () * 2);
    }
  txTime += GetResponseDuration (params, dataTxVector, item);

  return txTime;
}

Time
MacLow::CalculateTransmissionTime (Ptr<const Packet> packet,
                                   const WifiMacHeader* hdr,
                                   const MacLowTransmissionParameters& params) const
{
  Time txTime = CalculateOverallTxTime (packet, hdr, params);
  if (params.HasNextPacket ())
    {
      WifiTxVector dataTxVector = GetDataTxVector (Create<const WifiMacQueueItem> (packet, *hdr));
      txTime += GetSifs ();
      txTime += m_phy->CalculateTxDuration (params.GetNextPacketSize (), dataTxVector, m_phy->GetFrequency ());
    }
  return txTime;
}

void
MacLow::NotifyNav (Ptr<const Packet> packet, const WifiMacHeader &hdr)
{
  NS_ASSERT (m_lastNavStart <= Simulator::Now ());
  if (hdr.GetRawDuration () > 32767)
    {
      //All stations process Duration field values less than or equal to 32 767 from valid data frames
      //to update their NAV settings as appropriate under the coordination function rules.
      return;
    }
  if (hdr.IsCfEnd () && hdr.GetAddr2 () == m_bssid)
    {
      //see section 9.3.2.2 802.11-1999
      DoNavResetNow (Seconds (0));
      return;
    }
  else if (hdr.GetAddr1 () != m_self)
    {
      // see section 9.2.5.4 802.11-1999
      Time duration = hdr.GetDuration ();
      bool navUpdated = DoNavStartNow (duration);
      if (hdr.IsRts () && navUpdated)
        {
          /**
           * A STA that used information from an RTS frame as the most recent basis to update its NAV setting
           * is permitted to reset its NAV if no PHY-RXSTART.indication is detected from the PHY during a
           * period with a duration of (2 * aSIFSTime) + (CTS_Time) + (2 * aSlotTime) starting at the
           * PHY-RXEND.indication corresponding to the detection of the RTS frame. The “CTS_Time” shall
           * be calculated using the length of the CTS frame and the data rate at which the RTS frame
           * used for the most recent NAV update was received.
           */
          WifiMacHeader cts;
          cts.SetType (WIFI_MAC_CTL_CTS);
          WifiTxVector txVector = GetRtsTxVector (Create<const WifiMacQueueItem> (packet, hdr));
          Time navCounterResetCtsMissedDelay =
            m_phy->CalculateTxDuration (cts.GetSerializedSize (), txVector, m_phy->GetFrequency ()) +
            Time (2 * GetSifs ()) + Time (2 * GetSlotTime ());
          m_navCounterResetCtsMissed = Simulator::Schedule (navCounterResetCtsMissedDelay,
                                                            &MacLow::NavCounterResetCtsMissed, this,
                                                            Simulator::Now ());
        }
    }
}

void
MacLow::NavCounterResetCtsMissed (Time rtsEndRxTime)
{
  if (m_phy->GetLastRxStartTime () < rtsEndRxTime)
    {
      DoNavResetNow (Seconds (0));
    }
}

void
MacLow::DoNavResetNow (Time duration)
{
  NS_LOG_FUNCTION (this << duration);
  for (ChannelAccessManagersCI i = m_channelAccessManagers.begin (); i != m_channelAccessManagers.end (); i++)
    {
      (*i)->NotifyNavResetNow (duration);
    }
  m_lastNavStart = Simulator::Now ();
  m_lastNavDuration = duration;
}

bool
MacLow::DoNavStartNow (Time duration)
{
  for (ChannelAccessManagersCI i = m_channelAccessManagers.begin (); i != m_channelAccessManagers.end (); i++)
    {
      (*i)->NotifyNavStartNow (duration);
    }
  Time newNavEnd = Simulator::Now () + duration;
  Time oldNavEnd = m_lastNavStart + m_lastNavDuration;
  if (newNavEnd > oldNavEnd)
    {
      m_lastNavStart = Simulator::Now ();
      m_lastNavDuration = duration;
      return true;
    }
  return false;
}

void
MacLow::NotifyAckTimeoutStartNow (Time duration)
{
  for (ChannelAccessManagersCI i = m_channelAccessManagers.begin (); i != m_channelAccessManagers.end (); i++)
    {
      (*i)->NotifyAckTimeoutStartNow (duration);
    }
}

void
MacLow::NotifyAckTimeoutResetNow (void)
{
  for (ChannelAccessManagersCI i = m_channelAccessManagers.begin (); i != m_channelAccessManagers.end (); i++)
    {
      (*i)->NotifyAckTimeoutResetNow ();
    }
}

void
MacLow::NotifyCtsTimeoutStartNow (Time duration)
{
  for (ChannelAccessManagersCI i = m_channelAccessManagers.begin (); i != m_channelAccessManagers.end (); i++)
    {
      (*i)->NotifyCtsTimeoutStartNow (duration);
    }
}

void
MacLow::NotifyCtsTimeoutResetNow (void)
{
  for (ChannelAccessManagersCI i = m_channelAccessManagers.begin (); i != m_channelAccessManagers.end (); i++)
    {
      (*i)->NotifyCtsTimeoutResetNow ();
    }
}

void
MacLow::ForwardDown (Ptr<const WifiPsdu> psdu, WifiTxVector txVector)
{
  ForwardDown (WifiPsduMap ({std::make_pair (SU_STA_ID, psdu)}), txVector);
}

void
MacLow::ForwardDown (WifiPsduMap psduMap, WifiTxVector txVector)
{
  NS_LOG_FUNCTION (this << txVector);
  NS_ASSERT (!psduMap.empty ());

  for (auto& psdu : psduMap)
    {
      NS_ASSERT (psdu.second->GetNMpdus () > 0);
      const WifiMacHeader& hdr = (*psdu.second->begin ())->GetHeader ();

      NS_LOG_DEBUG ("send " << hdr.GetTypeString () <<
                    ", to=" << hdr.GetAddr1 () <<
                    ", size=" << psdu.second->GetSize () <<
                    ", duration=" << hdr.GetDuration () <<
                    ", seq=0x" << std::hex << hdr.GetSequenceControl () << std::dec);
    }

  if (psduMap.size () == 1)
    {
      Ptr<const WifiPsdu> psdu = psduMap.begin ()->second;
      const WifiMacHeader& hdr = (*psdu->begin ())->GetHeader ();

      if (hdr.IsCfPoll () && m_stationManager->GetPcfSupported ())
        {
          Simulator::Schedule (GetPifs () + m_phy->CalculateTxDuration (psdu->GetSize (), txVector, m_phy->GetFrequency ()), &MacLow::CfPollTimeout, this);
        }
      if (hdr.IsBeacon () && m_stationManager->GetPcfSupported ())
        {
          if (Simulator::Now () > m_lastBeacon + m_beaconInterval)
            {
              m_cfpForeshortening = (Simulator::Now () - m_lastBeacon - m_beaconInterval);
            }
          m_lastBeacon = Simulator::Now ();
        }
      else if (hdr.IsCfEnd () && m_stationManager->GetPcfSupported ())
        {
          m_cfpStart = NanoSeconds (0);
          m_cfpForeshortening = NanoSeconds (0);
          m_cfAckInfo.appendCfAck = false;
          m_cfAckInfo.expectCfAck = false;
        }
      else if (IsCfPeriod () && hdr.HasData ())
        {
          m_cfAckInfo.expectCfAck = true;
        }

      if (psdu->IsSingle ())
        {
          txVector.SetAggregation (true);
          NS_LOG_DEBUG ("Sending S-MPDU");
        }
      else if (psdu->IsAggregate ())
        {
          txVector.SetAggregation (true);
          NS_LOG_DEBUG ("Sending A-MPDU");
        }
      else
        {
          NS_LOG_DEBUG ("Sending non aggregate MPDU");
        }
    }
  else
    {
      txVector.SetAggregation (true);
      NS_LOG_DEBUG ("Sending " << psduMap.size () << " PSDUs");
    }

  for (auto& psdu : psduMap)
    {
      for (auto& mpdu : *PeekPointer (psdu.second))
        {
          if (mpdu->GetHeader ().IsQosData ())
            {
              GetEdca (mpdu->GetHeader ().GetQosTid ())->CompleteMpduTx (mpdu);
            }
        }
    }
  m_forwardDown (psduMap, txVector);
  m_phy->Send (psduMap, txVector);
}

void
MacLow::CfPollTimeout (void)
{
  NS_LOG_FUNCTION (this);
  //to be reworked
  bool busy = false;
  for (ChannelAccessManagersCI i = m_channelAccessManagers.begin (); i != m_channelAccessManagers.end (); i++)
    {
      busy = (*i)->IsBusy ();
    }
  if (!busy)
    {
      NS_ASSERT (m_currentTxop != 0);
      m_currentTxop->MissedCfPollResponse (m_cfAckInfo.expectCfAck);
      m_cfAckInfo.expectCfAck = false;
    }
}

void
MacLow::CtsTimeout (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("cts timeout");
  NS_ABORT_MSG_IF (m_currentPacket.size () > 1, "CTS Timeout when the current packet is an MU PPDU");
  /// \todo should check that there was no rx start before now.
  /// we should restart a new cts timeout now until the expected
  /// end of rx if there was a rx start before now.
  Ptr<WifiPsdu> psdu = m_currentPacket.begin ()->second;
  m_stationManager->ReportRtsFailed (psdu->GetAddr1 (), &psdu->GetHeader (0));

  Ptr<QosTxop> qosTxop = DynamicCast<QosTxop> (m_currentTxop);
  if (qosTxop != 0)
    {
      qosTxop->NotifyMissedCts (std::list<Ptr<WifiMacQueueItem>> (psdu->begin (), psdu->end ()));
    }
  else
    {
      m_currentTxop->MissedCts ();
    }
  m_currentTxop = 0;
}

void
MacLow::NormalAckTimeout (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("normal ack timeout");
  /// \todo should check that there was no rx start before now.
  /// we should restart a new ack timeout now until the expected
  /// end of rx if there was a rx start before now.
  Ptr<Txop> txop = m_currentTxop;
  m_currentTxop = 0;
  bool txSuccess = false;

  // A Normal Ack may be expected in response to an S-MPDU included in a DL MU PPDU.
  if (m_txParams.HasDlMuAckSequence () && m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_SU_FORMAT)
    {
      NS_ASSERT (m_txParams.GetStationsReceiveAckFrom ().size () == 1);

      // If m_continueTxopIfNoSuResponseAfterMuPpdu is true, we should not terminate the
      // TXOP (if any).
      if (m_continueTxopIfNoSuResponseAfterMuPpdu)
        {
          txSuccess = true;
        }

      // A missed Ack must be handled by the AC that transmitted the DL MU PPDU, hence we
      // need to notify the S-MPDU to the AC that transmitted the MU DL PPDU.
      auto psduIt = m_currentPacket.find (GetStaId (m_txParams.GetStationsReceiveAckFrom ().front ()));
      NS_ASSERT (psduIt != m_currentPacket.end ());
      NS_ASSERT (psduIt->second->IsSingle ());
      Ptr<WifiMacQueueItem> mpdu = *psduIt->second->begin ();
      NS_ASSERT (mpdu->GetHeader ().IsQosData ());
      Ptr<QosTxop> qosTxop = DynamicCast<QosTxop> (txop);
      NS_ASSERT (qosTxop != 0);
      qosTxop->UpdateCurrentPacket (mpdu);
    }

  txop->MissedAck (txSuccess);
}

void
MacLow::BlockAckTimeout (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("block ack timeout");
  Ptr<Txop> txop = m_currentTxop;
  m_currentTxop = 0;
  // The transmission of an MU PPDU is successful if at least one Block Ack is correctly
  // received. Determine if the transmission is successful despite some Block Acks are
  // lost and pass the result to the QosTxop, which will use such result to decide whether
  // to terminate the TXOP (if any) or not.
  bool txSuccess = false;
  auto psduMap = m_currentPacket;

  if (m_currentPacket.size () == 1 && m_currentPacket.begin ()->second->GetNMpdus () == 1
      && m_currentPacket.begin ()->second->GetHeader (0).IsTrigger ())
    {
      /* Acknowledgment via MU-BAR sent as SU PPDU, missed Block Ack(s) after MU-BAR */
      NS_ASSERT (!m_txParams.GetStationsReceiveBlockAckFrom ().empty ());
      CtrlTriggerHeader trigger;
      m_currentPacket.begin ()->second->GetPayload (0)->PeekHeader (trigger);
      NS_ASSERT (trigger.IsMuBar ());

      // the transmission is successful if the number of stations from which we did not
      // receive a Block Ack is less than the number of stations requested to send a Block Ack
      if (m_txParams.GetStationsReceiveBlockAckFrom ().size () < trigger.GetNUserInfoFields ())
        {
          txSuccess = true;
        }
    }
  else if (m_txParams.HasDlMuAckSequence () && m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF)
    {
      /* Acknowledgment via aggregated MU-BAR Trigger Frames, missed Block Ack(s) after MU DL PPDU */
      NS_ASSERT (!m_txParams.GetStationsReceiveBlockAckFrom ().empty ());

      // the transmission is successful if the number of stations from which we did not
      // receive a Block Ack is less than the number of stations requested to send a Block Ack
      if (m_txParams.GetStationsReceiveBlockAckFrom ().size () < m_currentPacket.size ())
        {
          txSuccess = true;
        }

      // QosTxop::MissedBlockAck will go through all the data frames that have not
      // been acknowledged. Hence, only add the unacknowledged ones to psduMap.
      psduMap.clear ();
      for (auto& sta : m_txParams.GetStationsReceiveBlockAckFrom ())
        {
          auto psduIt = m_currentPacket.find (GetStaId (sta));
          NS_ASSERT (psduIt != m_currentPacket.end ());
          psduMap.insert (*psduIt);
        }
    }
  else if (m_txParams.HasDlMuAckSequence () && m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_SU_FORMAT)
    {
      /* Acknowledgment in SU format, missed Block Ack after MU DL PPDU */
      NS_ASSERT (m_txParams.GetStationsReceiveBlockAckFrom ().size () == 1);

      // continue the TXOP if m_continueTxopIfNoSuResponseAfterMuPpdu is true
      if (m_continueTxopIfNoSuResponseAfterMuPpdu)
        {
          txSuccess = true;
        }

      // QosTxop::MissedBlockAck will go through all the data frames that have not
      // been acknowledged. Hence, only add the unacknowledged one to psduMap.
      psduMap.clear ();
      Mac48Address sta = m_txParams.GetStationsReceiveBlockAckFrom ().front ();
      auto psduIt = m_currentPacket.find (GetStaId (sta));
      NS_ASSERT (psduIt != m_currentPacket.end ());
      psduMap.insert (*psduIt);

      // A missed Block Ack must be handled by the AC that transmitted the DL MU PPDU,
      // hence we need to and notify the A-MPDU to the AC that transmitted the MU DL PPDU.
      Ptr<WifiMacQueueItem> mpdu = *psduIt->second->begin ();
      NS_ASSERT (mpdu->GetHeader ().IsQosData ());
      Ptr<QosTxop> qosTxop = DynamicCast<QosTxop> (txop);
      NS_ASSERT (qosTxop != 0);
      qosTxop->UpdateCurrentPacket (mpdu);
    }
  else if (m_ofdmaManager != 0 && m_continueTxopIfNoSuResponseAfterMuPpdu && txop->IsQosTxop ()
           && txop->GetTxopLimit ().IsStrictlyPositive () && txop->GetTxopRemaining () > GetSifs ()
           && m_ofdmaManager->GetTxFormat () == OfdmaTxFormat::DL_OFDMA
           && txop == m_ofdmaManager->GetDlOfdmaInfo ().txop)
    {
      // This block allows an AP to continue the TXOP even if a response expected in SU format
      // is not received after (but not immediately after) a transmission of a DL MU PPDU.
      // This requires that:
      // - m_continueTxopIfNoSuResponseAfterMuPpdu is true
      // - the AC expecting a response has a non-null TXOP Limit and there is enough remaining
      //   time (at least a SIFS) in the current TXOP
      // - the most recently transmitted QoS data frame is a DL MU PPDU and has been
      //   transmitted by the AC expecting a response
      // Also note that:
      // - the previous two blocks address the cases of missed responses in UL MU. Hence, we
      //   only get here if a response in SU format is missing
      // - the OFDMA Manager is only invoked when QoS data frames are to be sent. Hence,
      //   at any time it keeps the format of the last transmission of QoS data frames
      // - Block Ack Requests have precedence over data frames (based on the
      //   QosTxop behavior). Hence, a data frame can only be sent if there are no pending BARs
      // All of the above should guarantee that, if we get here, it is because a Block Ack
      // sent as SU PPDU in response to a DL MU PPDU (or in response to a Block Ack Request
      // sent after not all of the PSDUs sent in a DL MU PPDU were acknowledged) was not received.
      txSuccess = true;
    }

  txop->MissedBlockAck (psduMap, m_txParams, txSuccess);
}

void
MacLow::SendRtsForPacket (void)
{
  NS_LOG_FUNCTION (this);
  NS_ABORT_MSG_IF (m_currentPacket.size () > 1, "Sending RTS when the current packet is an MU PPDU");
  Ptr<WifiPsdu> psdu = m_currentPacket.begin ()->second;
  /* send an RTS for this packet. */
  WifiMacHeader rts;
  rts.SetType (WIFI_MAC_CTL_RTS);
  rts.SetDsNotFrom ();
  rts.SetDsNotTo ();
  rts.SetNoRetry ();
  rts.SetNoMoreFragments ();
  rts.SetAddr1 (psdu->GetAddr1 ());
  rts.SetAddr2 (m_self);
  WifiTxVector rtsTxVector = GetRtsTxVector (*psdu->begin ());
  Time duration = Seconds (0);

  duration += GetSifs ();
  duration += GetCtsDuration (psdu->GetAddr1 (), rtsTxVector);
  duration += GetSifs ();
  duration += m_phy->CalculateTxDuration (psdu->GetSize (),
                                          m_currentTxVector, m_phy->GetFrequency ());
  duration += GetResponseDuration (m_txParams, m_currentTxVector, *psdu->begin ());
  if (m_txParams.HasNextPacket ())
    {
      duration += m_phy->CalculateTxDuration (m_txParams.GetNextPacketSize (),
                                              m_currentTxVector, m_phy->GetFrequency ());
      duration += GetResponseDuration (m_txParams, m_currentTxVector, *psdu->begin ());
    }
  rts.SetDuration (duration);

  Time txDuration = m_phy->CalculateTxDuration (GetRtsSize (), rtsTxVector, m_phy->GetFrequency ());
  Time timerDelay = txDuration + GetCtsTimeout ();

  NS_ASSERT (m_ctsTimeoutEvent.IsExpired ());
  NotifyCtsTimeoutStartNow (timerDelay);
  m_ctsTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::CtsTimeout, this);

  ForwardDown (Create<const WifiPsdu> (Create<Packet> (), rts), rtsTxVector);
}

void
MacLow::StartDataTxTimers (WifiTxVector dataTxVector)
{
  NS_LOG_FUNCTION (this << dataTxVector);
  Time txDuration = m_phy->CalculateTxDuration (WifiPsduMap (m_currentPacket.begin (), m_currentPacket.end ()),
                                                dataTxVector, m_phy->GetFrequency ());

  if ((m_txParams.MustWaitNormalAck () && !IsCfPeriod ())
      || (m_txParams.HasDlMuAckSequence () && m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_SU_FORMAT
          && !m_txParams.GetStationsReceiveAckFrom ().empty ()))
    {
      // find the TX vector used to transmit the response (Normal Ack)
      WifiTxVector ackTxVector;

      if (m_txParams.MustWaitNormalAck ())
        {
          NS_ASSERT (m_currentPacket.size () == 1 && m_currentPacket.begin ()->second->GetNMpdus () == 1);
          ackTxVector = GetAckTxVector (m_currentPacket.begin ()->second->GetAddr1 (),
                                        dataTxVector.GetMode ());
        }
      else
        {
          auto txVectorUserInfo = dataTxVector.GetHeMuUserInfoMap ();
          Mac48Address receiver = m_txParams.GetStationsReceiveAckFrom ().front ();
          auto userInfoIt = txVectorUserInfo.find (GetStaId (receiver));
          NS_ASSERT (userInfoIt != txVectorUserInfo.end ());
          ackTxVector = GetAckTxVector (receiver, userInfoIt->second.mcs);
        }
      // the timeout duration is "aSIFSTime + aSlotTime + aRxPHYStartDelay, starting
      // at the PHY-TXEND.confirm primitive" (section 10.3.2.9 or 10.22.2.2 of 802.11-2016).
      // aRxPHYStartDelay equals the time to transmit the PHY header.
      Time timerDelay = txDuration + GetSifs () + GetSlotTime () + m_phy->CalculatePlcpPreambleAndHeaderDuration (ackTxVector);
      NS_ASSERT (m_normalAckTimeoutEvent.IsExpired ());
      NotifyAckTimeoutStartNow (timerDelay);
      m_normalAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::NormalAckTimeout, this);
    }
  else if (m_txParams.MustWaitBlockAck () || !m_txParams.GetStationsReceiveBlockAckFrom ().empty ())
    {
      // find the TX vector used to transmit the response (Block Ack)
      WifiTxVector blockAckTxVector;

      if (m_txParams.MustWaitBlockAck ())
        {
          NS_ASSERT (m_currentPacket.size () == 1);
          blockAckTxVector = GetBlockAckTxVector (m_currentPacket.begin ()->second->GetAddr1 (),
                                                  dataTxVector.GetMode ());
        }
      else if (m_txParams.HasDlMuAckSequence () && m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_SU_FORMAT)
        {
          /*
           * A PSDU within an MU DL PPDU is being sent with QoS Ack Policy equal
           * to Implicit BAR policy (acknowledgment in SU format)
           */
          Mac48Address receiver = m_txParams.GetStationsReceiveBlockAckFrom ().front ();
          blockAckTxVector = GetBlockAckTxVector (receiver, dataTxVector.GetMode (GetStaId (receiver)));
        }
      else if (m_currentPacket.size () == 1 && m_currentPacket.begin ()->second->GetNMpdus () == 1
               && m_currentPacket.begin ()->second->GetHeader (0).IsTrigger ())
        {
          /* A MU-BAR is being sent to request acknowledgment of an MU DL PPDU */
          CtrlTriggerHeader trigger;
          m_currentPacket.begin ()->second->GetPayload (0)->PeekHeader (trigger);
          blockAckTxVector = trigger.GetHeTbTxVector (GetStaId (m_txParams.GetStationsReceiveBlockAckFrom ().front ()));
        }
      else
        {
          /* Each PSDU in an MU DL PPDU includes a MU-BAR Trigger Frame (as last MPDU) */
          NS_ASSERT (m_txParams.HasDlMuAckSequence () && m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF);
          // The PHY headers of the Block Acks sent as HE TB PPDU are all the same.
          // Hence, it suffices to determine the TX vector used by one of them.
          CtrlTriggerHeader trigger;
          std::size_t lastMpdu = m_currentPacket.begin ()->second->GetNMpdus () - 1;
          m_currentPacket.begin ()->second->GetPayload (lastMpdu)->PeekHeader (trigger);
          blockAckTxVector = trigger.GetHeTbTxVector (m_currentPacket.begin ()->first);
        }

      // the timeout duration is "aSIFSTime + aSlotTime + aRxPHYStartDelay, starting
      // at the PHY-TXEND.confirm primitive" (section 10.3.2.9 or 10.22.2.2 of 802.11-2016).
      // aRxPHYStartDelay equals the time to transmit the PHY header.
      Time timerDelay = txDuration + GetSifs () + GetSlotTime () + m_phy->CalculatePlcpPreambleAndHeaderDuration (blockAckTxVector);
      NS_ASSERT (m_blockAckTimeoutEvent.IsExpired ());
      NotifyAckTimeoutStartNow (timerDelay);
      m_blockAckTimeoutEvent = Simulator::Schedule (timerDelay, &MacLow::BlockAckTimeout, this);
    }
  else if (m_txParams.HasNextPacket ())
    {
      NS_ASSERT (m_waitIfsEvent.IsExpired ());
      Time delay = txDuration;
      if (m_stationManager->GetRifsPermitted ())
        {
          delay += GetRifs ();
        }
      else
        {
          delay += GetSifs ();
        }
      m_waitIfsEvent = Simulator::Schedule (delay, &MacLow::WaitIfsAfterEndTxFragment, this);
    }
  else if (m_currentTxop->GetTxopLimit ().IsStrictlyPositive ()
           && m_currentTxop->GetTxopRemaining () - txDuration > GetSifs ())
   {
      Time delay = txDuration;
      if (m_stationManager->GetRifsPermitted ())
        {
          delay += GetRifs ();
        }
      else
        {
          delay += GetSifs ();
        }
      m_waitIfsEvent = Simulator::Schedule (delay, &MacLow::WaitIfsAfterEndTxPacket, this);
    }
  else
    {
      m_endTxNoAckEvent = Simulator::Schedule (txDuration, &MacLow::EndTxNoAck, this);
    }
}

void
MacLow::ScheduleBlockAckRequestsIfNeeded (void) const
{
  NS_LOG_FUNCTION (this);

  if (m_txParams.MustSendBlockAckRequest ())
    {
      // SU PPDU followed by a BAR
      NS_ASSERT (m_currentPacket.size () == 1);
      Ptr<QosTxop> qosTxop = DynamicCast<QosTxop> (m_currentTxop);
      NS_ASSERT (qosTxop != 0);
      auto bar = qosTxop->PrepareBlockAckRequest (m_currentPacket.begin ()->second->GetAddr1 (),
                                                  *m_currentPacket.begin ()->second->GetTids ().begin ());
      qosTxop->ScheduleBar (bar);
    }
  else if (m_txParams.HasDlMuAckSequence ())
    {
      // DL MU PPDU
      Ptr<QosTxop> qosTxop = DynamicCast<QosTxop> (m_currentTxop);
      NS_ASSERT (qosTxop != 0);
      std::list<Mac48Address> list = m_txParams.GetStationsSendBlockAckRequestTo ();

      // DL MU PPDU - Acknowledgment via a sequence of BAR/BA pairs
      if (m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_SU_FORMAT)
        {
          // schedule the transmission of multiple Block Ack Requests
          for (auto& psdu : m_currentPacket)
            {
              // search the receiver of this PSDU in the list of stations to which we send a BAR
              auto staIt = std::find (list.begin (), list.end (), psdu.second->GetAddr1 ());
              if (staIt != list.end ())
                {
                  NS_LOG_DEBUG ("Schedule Block Ack Request to " << psdu.second->GetAddr1 ());
                  auto bar = qosTxop->PrepareBlockAckRequest (psdu.second->GetAddr1 (),
                                                              *psdu.second->GetTids ().begin ());
                  qosTxop->ScheduleBar (bar);
                }
            }
        }
      // DL MU PPDU - Acknowledgment via a MU-BAR and Block Acks in a HE TB PPDU
      else if (m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_MU_BAR)
        {
          // schedule a MU BAR
          NS_ASSERT (m_ofdmaManager != 0);

          // build a Trigger frame that only contains the User Info fields corresponding to
          // stations that are actually supposed to be the receiver of the MU-BAR frame.
          // In fact, MacLow::StartTransmission may have not been able to include a PSDU
          // for all the stations returned by the OFDMA manager and not all the receivers
          // of a PSDU may be requested to respond with a Block Ack.
          std::map<uint16_t, std::pair<Mac48Address, uint8_t>> recipients;
          std::ostringstream msg;
          for (auto& psdu : m_currentPacket)
            {
              // search the receiver of this PSDU in the list of stations to which we send a BAR
              auto staIt = std::find (list.begin (), list.end (), psdu.second->GetAddr1 ());
              if (staIt != list.end ())
                {
                  recipients[GetStaId (psdu.second->GetAddr1 ())] = std::make_pair (psdu.second->GetAddr1 (),
                                                                                    *psdu.second->GetTids ().begin ());
                  msg << psdu.second->GetAddr1 () << ", ";
                }
            }

          NS_LOG_DEBUG ("Schedule MU BAR to " << msg.str ());
          CtrlTriggerHeader trigger = m_ofdmaManager->GetDlOfdmaInfo ().trigger;
          trigger.SetUlLength (CalculateUlLengthForBlockAcks (trigger, m_txParams));
          qosTxop->ScheduleBar (qosTxop->PrepareMuBar (trigger, recipients));
        }
    }
}

void
MacLow::SendDataPacket (void)
{
  NS_LOG_FUNCTION (this);
  /* send this packet directly. No RTS is needed. */
  StartDataTxTimers (m_currentTxVector);
  Ptr<WifiPsdu> firstPsdu = m_currentPacket.begin ()->second;

  if (!IsCfPeriod ())
    {
      Time duration = GetResponseDuration (m_txParams, m_currentTxVector, *firstPsdu->begin ());
      if (m_txParams.HasNextPacket ())
        {
          if (m_stationManager->GetRifsPermitted ())
            {
              duration += GetRifs ();
            }
          else
            {
              duration += GetSifs ();
            }
          duration += m_phy->CalculateTxDuration (m_txParams.GetNextPacketSize (),
                                                  m_currentTxVector, m_phy->GetFrequency ());
          duration += GetResponseDuration (m_txParams, m_currentTxVector, *firstPsdu->begin ());
        }
      for (auto& psdu : m_currentPacket)
        {
          psdu.second->SetDuration (duration);
        }
    }
  else
    {
      if (firstPsdu->GetHeader (0).IsCfEnd ())
        {
          firstPsdu->GetHeader (0).SetRawDuration (0);
        }
      else
        {
          firstPsdu->GetHeader (0).SetRawDuration (32768);
        }
    }

  if (!firstPsdu->IsAggregate ())
    {
      if (m_cfAckInfo.appendCfAck)
        {
          switch (firstPsdu->GetHeader (0).GetType ())
            {
            case WIFI_MAC_DATA:
              firstPsdu->GetHeader (0).SetType (WIFI_MAC_DATA_CFACK, false);
              break;
            case WIFI_MAC_DATA_CFPOLL:
              firstPsdu->GetHeader (0).SetType (WIFI_MAC_DATA_CFACK_CFPOLL, false);
              break;
            case WIFI_MAC_DATA_NULL:
              firstPsdu->GetHeader (0).SetType (WIFI_MAC_DATA_NULL_CFACK, false);
              break;
            case WIFI_MAC_DATA_NULL_CFPOLL:
              firstPsdu->GetHeader (0).SetType (WIFI_MAC_DATA_NULL_CFACK_CFPOLL, false);
              break;
            case WIFI_MAC_CTL_END:
              firstPsdu->GetHeader (0).SetType (WIFI_MAC_CTL_END_ACK, false);
              break;
            default:
              NS_ASSERT (false);
              break;
            }
          NS_ASSERT (m_cfAckInfo.address != Mac48Address ());
          //Standard says that, for frames of type Data+CF-ACK, Data+CF-Poll+CF-ACK, and CF-Poll+CF-ACK,
          //the rate chosen to transmit the frame must be supported by both the addressed recipient STA and the STA to which the ACK is intended.
          //This ideally requires the rate manager to handle this case, but this requires to update all rate manager classes.
          //Instead, we simply fetch two TxVector and we select the one with the lowest datarate.
          //This should be later changed, at the latest once HCCA is implemented for HT/VHT/HE stations.
          WifiMacHeader tmpHdr = firstPsdu->GetHeader (0);
          tmpHdr.SetAddr1 (m_cfAckInfo.address);
          WifiTxVector tmpTxVector = GetDataTxVector (Create<const WifiMacQueueItem> (firstPsdu->GetPayload (0), tmpHdr));
          if (tmpTxVector.GetMode ().GetDataRate (tmpTxVector) < m_currentTxVector.GetMode ().GetDataRate (m_currentTxVector))
            {
              m_currentTxVector = tmpTxVector;
            }
          m_cfAckInfo.appendCfAck = false;
          m_cfAckInfo.address = Mac48Address ();
        }
    }

  // Schedule the transmission of Block Ack Request(s)
  ScheduleBlockAckRequestsIfNeeded ();

  // Forward down every PSDU
  ForwardDown (WifiPsduMap (m_currentPacket.begin (), m_currentPacket.end ()), m_currentTxVector);
}

bool
MacLow::IsNavZero (void) const
{
  return (m_lastNavStart + m_lastNavDuration < Simulator::Now ());
}

void
MacLow::SendCtsToSelf (void)
{
  NS_ABORT_MSG_IF (m_currentPacket.size () > 1, "Sending CTS-to-self when the current packet is an MU PPDU");
  Ptr<WifiPsdu> psdu = m_currentPacket.begin ()->second;
  WifiMacHeader cts;
  cts.SetType (WIFI_MAC_CTL_CTS);
  cts.SetDsNotFrom ();
  cts.SetDsNotTo ();
  cts.SetNoMoreFragments ();
  cts.SetNoRetry ();
  cts.SetAddr1 (m_self);

  WifiTxVector ctsTxVector = GetRtsTxVector (*psdu->begin ());
  Time duration = Seconds (0);

  duration += GetSifs ();
  duration += m_phy->CalculateTxDuration (psdu->GetSize (),
                                          m_currentTxVector, m_phy->GetFrequency ());
  duration += GetResponseDuration (m_txParams, m_currentTxVector, *psdu->begin ());
  if (m_txParams.HasNextPacket ())
    {
      duration += GetSifs ();
      duration += m_phy->CalculateTxDuration (m_txParams.GetNextPacketSize (),
                                              m_currentTxVector, m_phy->GetFrequency ());
      duration += GetResponseDuration (m_txParams, m_currentTxVector, *psdu->begin ());
    }

  cts.SetDuration (duration);

  ForwardDown (Create<const WifiPsdu> (Create<Packet> (), cts), ctsTxVector);

  Time txDuration = m_phy->CalculateTxDuration (GetCtsSize (), ctsTxVector, m_phy->GetFrequency ());
  txDuration += GetSifs ();
  NS_ASSERT (m_sendDataEvent.IsExpired ());

  m_sendDataEvent = Simulator::Schedule (txDuration,
                                         &MacLow::SendDataAfterCts, this,
                                         duration);
}

void
MacLow::SendCtsAfterRts (Mac48Address source, Time duration, WifiTxVector rtsTxVector, double rtsSnr)
{
  NS_LOG_FUNCTION (this << source << duration << rtsTxVector.GetMode () << rtsSnr);
  /* send a CTS when you receive a RTS
   * right after SIFS.
   */
  WifiTxVector ctsTxVector = GetCtsTxVector (source, rtsTxVector.GetMode ());
  WifiMacHeader cts;
  cts.SetType (WIFI_MAC_CTL_CTS);
  cts.SetDsNotFrom ();
  cts.SetDsNotTo ();
  cts.SetNoMoreFragments ();
  cts.SetNoRetry ();
  cts.SetAddr1 (source);
  duration -= GetCtsDuration (source, rtsTxVector);
  duration -= GetSifs ();
  NS_ASSERT (duration.IsPositive ());
  cts.SetDuration (duration);

  Ptr<Packet> packet = Create<Packet> ();

  SnrTag tag;
  tag.Set (rtsSnr);
  packet->AddPacketTag (tag);

  //CTS should always use non-HT PPDU (HT PPDU cases not supported yet)
  ForwardDown (Create<const WifiPsdu> (packet, cts), ctsTxVector);
}

void
MacLow::SendDataAfterCts (Time duration)
{
  NS_LOG_FUNCTION (this);
  /* send the third step in a
   * RTS/CTS/DATA/ACK handshake
   */
  NS_ASSERT (m_currentPacket.size () == 1);
  Ptr<WifiPsdu> psdu = m_currentPacket.begin ()->second;

  StartDataTxTimers (m_currentTxVector);
  Time newDuration = GetResponseDuration (m_txParams, m_currentTxVector, *psdu->begin ());
  if (m_txParams.HasNextPacket ())
    {
      if (m_stationManager->GetRifsPermitted ())
        {
          newDuration += GetRifs ();
        }
      else
        {
          newDuration += GetSifs ();
        }
      newDuration += m_phy->CalculateTxDuration (m_txParams.GetNextPacketSize (), m_currentTxVector, m_phy->GetFrequency ());
      newDuration += GetResponseDuration (m_txParams, m_currentTxVector, *psdu->begin ());
    }

  Time txDuration = m_phy->CalculateTxDuration (psdu->GetSize (), m_currentTxVector, m_phy->GetFrequency ());
  duration -= txDuration;
  duration -= GetSifs ();

  duration = std::max (duration, newDuration);
  NS_ASSERT (duration.IsPositive ());
  for (auto& psdu : m_currentPacket)
    {
      psdu.second->SetDuration (duration);
    }

  // Schedule the transmission of Block Ack Request(s)
  ScheduleBlockAckRequestsIfNeeded ();

  // Forward down every PSDU
  ForwardDown (WifiPsduMap (m_currentPacket.begin (), m_currentPacket.end ()), m_currentTxVector);
}

void
MacLow::WaitIfsAfterEndTxFragment (void)
{
  NS_LOG_FUNCTION (this);
  m_currentTxop->StartNextFragment ();
}

void
MacLow::WaitIfsAfterEndTxPacket (void)
{
  NS_LOG_FUNCTION (this);
  m_currentTxop->StartNextPacket ();
}

void
MacLow::EndTxNoAck (void)
{
  NS_LOG_FUNCTION (this);
  if (m_currentPacket.begin ()->second->GetHeader (0).IsBeacon () && m_stationManager->GetPcfSupported ())
    {
      m_cfpStart = Simulator::Now ();
    }
  if (!m_cfAckInfo.expectCfAck)
    {
      Ptr<Txop> txop = m_currentTxop;
      txop->EndTxNoAck ();
    }
  if (!IsCfPeriod ())
    {
      m_currentTxop = 0;
    }
}

void
MacLow::SendAckAfterData (Mac48Address source, Time duration, WifiMode dataTxMode, double dataSnr)
{
  NS_LOG_FUNCTION (this);
  // send an ACK, after SIFS, when you receive a packet
  WifiTxVector ackTxVector = GetAckTxVector (source, dataTxMode);
  WifiMacHeader ack;
  ack.SetType (WIFI_MAC_CTL_ACK);
  ack.SetDsNotFrom ();
  ack.SetDsNotTo ();
  ack.SetNoRetry ();
  ack.SetNoMoreFragments ();
  ack.SetAddr1 (source);
  // 802.11-2012, Section 8.3.1.4:  Duration/ID is received duration value
  // minus the time to transmit the ACK frame and its SIFS interval
  duration -= GetAckDuration (ackTxVector);
  duration -= GetSifs ();
  NS_ASSERT_MSG (duration.IsPositive (), "Please provide test case to maintainers if this assert is hit.");
  ack.SetDuration (duration);

  Ptr<Packet> packet = Create<Packet> ();

  SnrTag tag;
  tag.Set (dataSnr);
  packet->AddPacketTag (tag);

  //ACK should always use non-HT PPDU (HT PPDU cases not supported yet)
  ForwardDown (Create<const WifiPsdu> (packet, ack), ackTxVector);
}

bool
MacLow::ReceiveMpdu (Ptr<WifiMacQueueItem> mpdu)
{
  const WifiMacHeader& hdr = mpdu->GetHeader ();

  if (m_stationManager->GetHtSupported ()
      || m_stationManager->GetVhtSupported ()
      || m_stationManager->GetHeSupported ())
    {
      Mac48Address originator = hdr.GetAddr2 ();
      uint8_t tid = 0;
      if (hdr.IsQosData ())
        {
          tid = hdr.GetQosTid ();
        }
      uint16_t seqNumber = hdr.GetSequenceNumber ();
      AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
      if (it != m_bAckAgreements.end ())
        {
          //Implement HT immediate Block Ack support for HT Delayed Block Ack is not added yet
          if (!QosUtilsIsOldPacket ((*it).second.first.GetStartingSequence (), seqNumber))
            {
              StoreMpduIfNeeded (mpdu);
              if (!IsInWindow (hdr.GetSequenceNumber (), (*it).second.first.GetStartingSequence (), (*it).second.first.GetBufferSize ()))
                {
                  uint16_t delta = (seqNumber - (*it).second.first.GetWinEnd () + 4096) % 4096;
                  if (delta > 1)
                    {
                      (*it).second.first.SetWinEnd (seqNumber);
                      int16_t winEnd = (*it).second.first.GetWinEnd ();
                      int16_t bufferSize = (*it).second.first.GetBufferSize ();
                      uint16_t sum = (static_cast<uint16_t> (winEnd - bufferSize + 1 + 4096)) % 4096;
                      (*it).second.first.SetStartingSequence (sum);
                      RxCompleteBufferedPacketsWithSmallerSequence ((*it).second.first.GetStartingSequenceControl (), originator, tid);
                    }
                }
              RxCompleteBufferedPacketsUntilFirstLost (originator, tid); //forwards up packets starting from winstart and set winstart to last +1
              (*it).second.first.SetWinEnd (((*it).second.first.GetStartingSequence () + (*it).second.first.GetBufferSize () - 1) % 4096);
            }
          return true;
        }
      return false;
    }
  return StoreMpduIfNeeded (mpdu);
}

bool
MacLow::StoreMpduIfNeeded (Ptr<WifiMacQueueItem> mpdu)
{
  const WifiMacHeader& hdr = mpdu->GetHeader ();

  AgreementsI it = m_bAckAgreements.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
  if (it != m_bAckAgreements.end ())
    {
      uint16_t endSequence = ((*it).second.first.GetStartingSequence () + 2047) % 4096;
      uint32_t mappedSeqControl = QosUtilsMapSeqControlToUniqueInteger (hdr.GetSequenceControl (), endSequence);

      BufferedPacketI i = (*it).second.second.begin ();
      for (; i != (*it).second.second.end ()
           && QosUtilsMapSeqControlToUniqueInteger ((*i)->GetHeader ().GetSequenceControl (), endSequence) < mappedSeqControl; i++)
        {
        }
      (*it).second.second.insert (i, mpdu);

      //Update block ack cache
      BlockAckCachesI j = m_bAckCaches.find (std::make_pair (hdr.GetAddr2 (), hdr.GetQosTid ()));
      NS_ASSERT (j != m_bAckCaches.end ());
      (*j).second.UpdateWithMpdu (&hdr);
      return true;
    }
  return false;
}

void
MacLow::CreateBlockAckAgreement (const MgtAddBaResponseHeader *respHdr, Mac48Address originator,
                                 uint16_t startingSeq)
{
  NS_LOG_FUNCTION (this);
  uint8_t tid = respHdr->GetTid ();
  BlockAckAgreement agreement (originator, tid);
  if (respHdr->IsImmediateBlockAck ())
    {
      agreement.SetImmediateBlockAck ();
    }
  else
    {
      agreement.SetDelayedBlockAck ();
    }
  agreement.SetAmsduSupport (respHdr->IsAmsduSupported ());
  agreement.SetBufferSize (respHdr->GetBufferSize () + 1);
  agreement.SetTimeout (respHdr->GetTimeout ());
  agreement.SetStartingSequence (startingSeq);
  agreement.SetHtSupported (m_stationManager->GetHtSupported () && m_stationManager->GetHtSupported (originator));

  std::list<Ptr<WifiMacQueueItem>> buffer (0);
  AgreementKey key (originator, respHdr->GetTid ());
  AgreementValue value (agreement, buffer);
  m_bAckAgreements.insert (std::make_pair (key, value));

  BlockAckCache cache;
  cache.Init (startingSeq, respHdr->GetBufferSize () + 1);
  m_bAckCaches.insert (std::make_pair (key, cache));

  if (respHdr->GetTimeout () != 0)
    {
      AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, respHdr->GetTid ()));
      Time timeout = MicroSeconds (1024 * agreement.GetTimeout ());

      AcIndex ac = QosUtilsMapTidToAc (agreement.GetTid ());

      it->second.first.m_inactivityEvent = Simulator::Schedule (timeout,
                                                                &QosTxop::SendDelbaFrame,
                                                                m_edca[ac], originator, tid, false);
    }
}

void
MacLow::DestroyBlockAckAgreement (Mac48Address originator, uint8_t tid)
{
  NS_LOG_FUNCTION (this);
  AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
  if (it != m_bAckAgreements.end ())
    {
      RxCompleteBufferedPacketsWithSmallerSequence (it->second.first.GetStartingSequenceControl (), originator, tid);
      RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
      m_bAckAgreements.erase (it);
      BlockAckCachesI i = m_bAckCaches.find (std::make_pair (originator, tid));
      NS_ASSERT (i != m_bAckCaches.end ());
      m_bAckCaches.erase (i);
    }
}

void
MacLow::RxCompleteBufferedPacketsWithSmallerSequence (uint16_t seq, Mac48Address originator, uint8_t tid)
{
  AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
  if (it != m_bAckAgreements.end ())
    {
      uint16_t endSequence = ((*it).second.first.GetStartingSequence () + 2047) % 4096;
      uint32_t mappedStart = QosUtilsMapSeqControlToUniqueInteger (seq, endSequence);
      BufferedPacketI last = (*it).second.second.begin ();
      uint16_t guard = 0;
      if (last != (*it).second.second.end ())
        {
          guard = (*(*it).second.second.begin ())->GetHeader ().GetSequenceControl ();
        }
      BufferedPacketI i = (*it).second.second.begin ();
      for (; i != (*it).second.second.end ()
           && QosUtilsMapSeqControlToUniqueInteger ((*i)->GetHeader ().GetSequenceControl (), endSequence) < mappedStart; )
        {
          if (guard == (*i)->GetHeader ().GetSequenceControl ())
            {
              if (!(*i)->GetHeader ().IsMoreFragments ())
                {
                  while (last != i)
                    {
                      m_rxCallback (*last);
                      last++;
                    }
                  m_rxCallback (*last);
                  last++;
                  /* go to next packet */
                  while (i != (*it).second.second.end () && guard == (*i)->GetHeader ().GetSequenceControl ())
                    {
                      i++;
                    }
                  if (i != (*it).second.second.end ())
                    {
                      guard = (*i)->GetHeader ().GetSequenceControl ();
                      last = i;
                    }
                }
              else
                {
                  guard++;
                }
            }
          else
            {
              /* go to next packet */
              while (i != (*it).second.second.end () && guard == (*i)->GetHeader ().GetSequenceControl ())
                {
                  i++;
                }
              if (i != (*it).second.second.end ())
                {
                  guard = (*i)->GetHeader ().GetSequenceControl ();
                  last = i;
                }
            }
        }
      (*it).second.second.erase ((*it).second.second.begin (), i);
    }
}

void
MacLow::RxCompleteBufferedPacketsUntilFirstLost (Mac48Address originator, uint8_t tid)
{
  AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
  if (it != m_bAckAgreements.end ())
    {
      uint16_t guard = (*it).second.first.GetStartingSequenceControl ();
      BufferedPacketI lastComplete = (*it).second.second.begin ();
      BufferedPacketI i = (*it).second.second.begin ();
      for (; i != (*it).second.second.end () && guard == (*i)->GetHeader ().GetSequenceControl (); i++)
        {
          if (!(*i)->GetHeader ().IsMoreFragments ())
            {
              while (lastComplete != i)
                {
                  m_rxCallback (*lastComplete);
                  lastComplete++;
                }
              m_rxCallback (*lastComplete);
              lastComplete++;
            }
          guard = (*i)->GetHeader ().IsMoreFragments () ? (guard + 1) : ((guard + 16) & 0xfff0);
        }
      (*it).second.first.SetStartingSequenceControl (guard);
      /* All packets already forwarded to WifiMac must be removed from buffer:
      [begin (), lastComplete) */
      (*it).second.second.erase ((*it).second.second.begin (), lastComplete);
    }
}

void
MacLow::SendBlockAckResponse (const CtrlBAckResponseHeader* blockAck, Mac48Address originator, bool immediate,
                              Time duration, WifiTxVector blockAckTxVector, double rxSnr)
{
  NS_LOG_FUNCTION (this);
  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (*blockAck);

  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_CTL_BACKRESP);
  hdr.SetAddr1 (originator);
  hdr.SetAddr2 (GetAddress ());
  hdr.SetDsNotFrom ();
  hdr.SetDsNotTo ();
  hdr.SetNoRetry ();
  hdr.SetNoMoreFragments ();

  if (immediate)
    {
      m_txParams.DisableAck ();
      duration -= GetSifs ();
      duration -= GetBlockAckDuration (blockAckTxVector, blockAck->GetType (), GetStaId (m_self));
    }
  else
    {
      m_txParams.EnableAck ();
      duration += GetSifs ();
      duration += GetAckDuration (originator, blockAckTxVector);
    }
  m_txParams.DisableNextData ();

  if (!immediate)
    {
      StartDataTxTimers (blockAckTxVector);
    }

  NS_ASSERT (duration.IsPositive ());
  hdr.SetDuration (duration);
  //here should be present a control about immediate or delayed block ack
  //for now we assume immediate
  SnrTag tag;
  tag.Set (rxSnr);
  packet->AddPacketTag (tag);
  uint16_t staId = (blockAckTxVector.GetPreambleType () == WIFI_PREAMBLE_HE_TB ? GetStaId (m_self) : SU_STA_ID);
  ForwardDown (WifiPsduMap ({std::make_pair (staId, Create<const WifiPsdu> (packet, hdr))}), blockAckTxVector);
}

void
MacLow::SendBlockAckAfterAmpdu (uint8_t tid, Mac48Address originator, Time duration, WifiTxVector dataTxVector, double rxSnr)
{
  NS_LOG_FUNCTION (this);
  if (!m_phy->IsStateTx () && !m_phy->IsStateRx ())
    {
      NS_LOG_FUNCTION (this << +tid << originator << duration.As (Time::S) << dataTxVector << rxSnr);
      CtrlBAckResponseHeader blockAck;
      uint16_t seqNumber = 0;
      BlockAckCachesI i = m_bAckCaches.find (std::make_pair (originator, tid));
      NS_ASSERT (i != m_bAckCaches.end ());
      seqNumber = (*i).second.GetWinStart ();

      bool immediate = true;
      AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
      blockAck.SetStartingSequence (seqNumber);
      blockAck.SetTidInfo (tid);
      immediate = (*it).second.first.IsImmediateBlockAck ();
      blockAck.SetType ((*it).second.first.GetBlockAckType ());
      NS_LOG_DEBUG ("Got Implicit block Ack Req with seq " << seqNumber);
      (*i).second.FillBlockAckBitmap (&blockAck);

      WifiTxVector blockAckTxVector = GetBlockAckTxVector (originator, dataTxVector.GetMode (GetStaId (m_self)));

      SendBlockAckResponse (&blockAck, originator, immediate, duration, blockAckTxVector, rxSnr);
    }
  else
    {
      NS_LOG_DEBUG ("Skip block ack response!");
    }
}

void
MacLow::SendBlockAckAfterBlockAckRequest (const CtrlBAckRequestHeader reqHdr, Mac48Address originator,
                                          Time duration, WifiTxVector blockAckTxVector, double rxSnr)
{
  NS_LOG_FUNCTION (this);
  CtrlBAckResponseHeader blockAck;
  uint8_t tid = 0;
  bool immediate = false;
  if (!reqHdr.IsMultiTid ())
    {
      tid = reqHdr.GetTidInfo ();
      AgreementsI it = m_bAckAgreements.find (std::make_pair (originator, tid));
      if (it != m_bAckAgreements.end ())
        {
          blockAck.SetStartingSequence (reqHdr.GetStartingSequence ());
          blockAck.SetTidInfo (tid);
          immediate = (*it).second.first.IsImmediateBlockAck ();
          blockAck.SetType ((*it).second.first.GetBlockAckType ());
          BlockAckCachesI i = m_bAckCaches.find (std::make_pair (originator, tid));
          NS_ASSERT (i != m_bAckCaches.end ());
          (*i).second.FillBlockAckBitmap (&blockAck);
          NS_LOG_DEBUG ("Got block Ack Req with seq " << reqHdr.GetStartingSequence ());

          if (!m_stationManager->GetHtSupported ()
              && !m_stationManager->GetVhtSupported ()
              && !m_stationManager->GetHeSupported ())
            {
              /* All packets with smaller sequence than starting sequence control must be passed up to Wifimac
               * See 9.10.3 in IEEE 802.11e standard.
               */
              RxCompleteBufferedPacketsWithSmallerSequence (reqHdr.GetStartingSequenceControl (), originator, tid);
              RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
            }
          else
            {
              if (!QosUtilsIsOldPacket ((*it).second.first.GetStartingSequence (), reqHdr.GetStartingSequence ()))
                {
                  (*it).second.first.SetStartingSequence (reqHdr.GetStartingSequence ());
                  (*it).second.first.SetWinEnd (((*it).second.first.GetStartingSequence () + (*it).second.first.GetBufferSize () - 1) % 4096);
                  RxCompleteBufferedPacketsWithSmallerSequence (reqHdr.GetStartingSequenceControl (), originator, tid);
                  RxCompleteBufferedPacketsUntilFirstLost (originator, tid);
                  (*it).second.first.SetWinEnd (((*it).second.first.GetStartingSequence () + (*it).second.first.GetBufferSize () - 1) % 4096);
                }
            }
        }
      else
        {
          NS_LOG_DEBUG ("there's not a valid block ack agreement with " << originator);
        }
    }
  else
    {
      NS_FATAL_ERROR ("Multi-tid block ack is not supported.");
    }
  SendBlockAckResponse (&blockAck, originator, immediate, duration, blockAckTxVector, rxSnr);
}

void
MacLow::ResetBlockAckInactivityTimerIfNeeded (BlockAckAgreement &agreement)
{
  if (agreement.GetTimeout () != 0)
    {
      NS_ASSERT (agreement.m_inactivityEvent.IsRunning ());
      agreement.m_inactivityEvent.Cancel ();
      Time timeout = MicroSeconds (1024 * agreement.GetTimeout ());
      AcIndex ac = QosUtilsMapTidToAc (agreement.GetTid ());
      agreement.m_inactivityEvent = Simulator::Schedule (timeout,
                                                         &QosTxop::SendDelbaFrame,
                                                         m_edca[ac], agreement.GetPeer (),
                                                         agreement.GetTid (), false);
    }
}

void
MacLow::RegisterEdcaForAc (AcIndex ac, Ptr<QosTxop> edca)
{
  m_edca.insert (std::make_pair (ac, edca));
}

void
MacLow::DeaggregateAmpduAndReceive (Ptr<WifiPsdu> psdu, RxSignalInfo rxSignalInfo,
                                    WifiTxVector txVector, std::vector<bool> statusPerMpdu)
{
  NS_LOG_FUNCTION (this << psdu << rxSignalInfo << txVector <<
                   statusPerMpdu.size () << std::all_of(statusPerMpdu.begin(), statusPerMpdu.end(), [](bool v) { return v; })); //returns true if all true
  bool normalAck = false;
  bool ampduSubframe = txVector.IsAggregation (); //flag indicating the packet belongs to an A-MPDU and is not a VHT/HE single MPDU
  //statusPerMpdu is empty for intermediate MPDU forwarding.
  //This function is called also once the PPDU has been fully received by the PHY,
  //in that case statusPerMpdu carries the information about the received MPDUs.
  if (ampduSubframe && !statusPerMpdu.empty ())
    {
      //We are here if a S-MPDU is received or if all MPDUs of an A-MPDU have been
      //received (but have been already forwarded up, so ReceiveOk won't be called)
      NS_ASSERT (psdu->IsAggregate ());
      ampduSubframe = true;
      auto n = psdu->begin ();
      auto status = statusPerMpdu.begin ();
      NS_ABORT_MSG_IF (psdu->GetNMpdus () != statusPerMpdu.size (), "Should have one receive status per MPDU");

      WifiMacHeader firsthdr = (*n)->GetHeader ();
      if (firsthdr.GetAddr1 () == m_self)
        {
          //Iterate over all MPDUs and notify reception only if status OK
          for (; n != psdu->end (); ++n, ++status)
            {
              firsthdr = (*n)->GetHeader ();
              NS_ABORT_MSG_IF (firsthdr.GetAddr1 () != m_self, "All MPDUs of A-MPDU should have the same destination address");
              if (*status) //PER and thus CRC check succeeded
                {
                  if (psdu->IsSingle ())
                    {
                      //If the MPDU is sent as a VHT/HE single MPDU (EOF=1 in A-MPDU subframe header), then the responder sends an ACK.
                      NS_LOG_DEBUG ("Receive S-MPDU");
                      ampduSubframe = false;
                    }
                  else if (!m_sendAckEvent.IsRunning () && firsthdr.IsQosData () && firsthdr.IsQosAck ()) // Implicit BAR Ack Policy
                    {
                      m_sendAckEvent = Simulator::Schedule (GetSifs (),
                                                            &MacLow::SendBlockAckAfterAmpdu, this,
                                                            firsthdr.GetQosTid (),
                                                            firsthdr.GetAddr2 (),
                                                            firsthdr.GetDuration (),
                                                            txVector, rxSignalInfo.snr);
                    }

                  if (firsthdr.IsAck () || firsthdr.IsBlockAck () || firsthdr.IsBlockAckReq () || firsthdr.IsTrigger ())
                    {
                      ReceiveOk ((*n), rxSignalInfo, txVector, ampduSubframe);
                    }
                  else if (firsthdr.IsData () || firsthdr.IsQosData ())
                    {
                      NS_LOG_DEBUG ("Deaggregate packet from " << firsthdr.GetAddr2 () << " with sequence=" << firsthdr.GetSequenceNumber ());
                      if (psdu->IsSingle ())
                        {
                          ReceiveOk ((*n), rxSignalInfo, txVector, ampduSubframe);
                        }
                      if (firsthdr.IsQosAck ())
                        {
                          NS_LOG_DEBUG ("Normal Ack");
                          normalAck = true;
                        }
                    }
                  else
                    {
                      NS_FATAL_ERROR ("Received A-MPDU with invalid first MPDU type");
                    }

                  if (!psdu->IsSingle ())
                    {
                      if (normalAck)
                        {
                          //send block Ack
                          if (firsthdr.IsBlockAckReq ())
                            {
                              NS_FATAL_ERROR ("Sending a BlockAckReq with QosPolicy equal to Normal Ack");
                            }
                          uint8_t tid = firsthdr.GetQosTid ();
                          AgreementsI it = m_bAckAgreements.find (std::make_pair (firsthdr.GetAddr2 (), tid));
                          if (it != m_bAckAgreements.end ())
                            {
                              /* See section 11.5.3 in IEEE 802.11 for mean of this timer */
                              ResetBlockAckInactivityTimerIfNeeded (it->second.first);
                              NS_LOG_DEBUG ("rx A-MPDU/sendImmediateBlockAck from=" << firsthdr.GetAddr2 ());
                              NS_ASSERT (m_sendAckEvent.IsRunning ());
                            }
                          else
                            {
                              NS_LOG_DEBUG ("There's not a valid agreement for this block ack request.");
                            }
                        }
                    }
                }
            }
        }
    }
  else
    {
      /* An MPDU has been received */
      NS_ASSERT (!psdu->IsAggregate ());
      ReceiveOk ((*psdu->begin ()), rxSignalInfo, txVector, ampduSubframe);
    }
}

Time
MacLow::GetRemainingCfpDuration (void) const
{
  NS_LOG_FUNCTION (this);
  Time remainingCfpDuration = std::min (m_cfpStart, m_cfpStart + m_cfpMaxDuration - Simulator::Now () - m_cfpForeshortening);
  NS_ASSERT (remainingCfpDuration.IsPositive ());
  return remainingCfpDuration;
}

bool
MacLow::IsCfPeriod (void) const
{
  return (m_stationManager->GetPcfSupported () && m_cfpStart.IsStrictlyPositive ());
}

bool
MacLow::CanTransmitNextCfFrame (void) const
{
  NS_LOG_FUNCTION (this);
  if (!IsCfPeriod ())
    {
      return false;
    }
  NS_ASSERT (GetRemainingCfpDuration ().IsPositive ());
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_DATA);
  WifiMacTrailer fcs;
  uint32_t maxMacFrameSize = MAX_MSDU_SIZE + hdr.GetSerializedSize () + fcs.GetSerializedSize ();
  Time nextTransmission = 2 * m_phy->CalculateTxDuration (maxMacFrameSize, m_currentTxVector, m_phy->GetFrequency ()) + 3 * GetSifs () + m_phy->CalculateTxDuration (GetCfEndSize (), m_currentTxVector, m_phy->GetFrequency ());
  return ((GetRemainingCfpDuration () - nextTransmission).IsPositive ());
}

uint16_t
MacLow::GetStaId (Mac48Address receiver) const
{
  Ptr<ApWifiMac> apMac = DynamicCast<ApWifiMac> (m_mac);
  if (apMac != 0)
    {
      return apMac->GetAssociationId (receiver);
    }
  Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac> (m_mac);
  if (staMac != 0 && staMac->IsAssociated ())
    {
      return staMac->GetAssociationId ();
    }
  return SU_STA_ID;
}

} //namespace ns3
