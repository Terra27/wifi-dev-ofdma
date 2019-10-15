/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006, 2009 INRIA
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
 *          Stefano Avallone <stavalli@unina.it>
 */

#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/random-variable-stream.h"
#include "qos-txop.h"
#include "channel-access-manager.h"
#include "mac-tx-middle.h"
#include "mgt-headers.h"
#include "wifi-mac-trailer.h"
#include "wifi-mac-queue.h"
#include "mac-low.h"
#include "qos-blocked-destinations.h"
#include "wifi-remote-station-manager.h"
#include "msdu-aggregator.h"
#include "mpdu-aggregator.h"
#include "ctrl-headers.h"
#include "wifi-phy.h"
#include "wifi-ack-policy-selector.h"
#include "wifi-psdu.h"
#include "wifi-net-device.h"
#include "ap-wifi-mac.h"

#undef NS_LOG_APPEND_CONTEXT
#define NS_LOG_APPEND_CONTEXT if (m_low != 0) { std::clog << "[mac=" << m_low->GetAddress () << "] "; }

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("QosTxop");

NS_OBJECT_ENSURE_REGISTERED (QosTxop);

TypeId
QosTxop::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::QosTxop")
    .SetParent<ns3::Txop> ()
    .SetGroupName ("Wifi")
    .AddConstructor<QosTxop> ()
    .AddAttribute ("UseExpliciteBarAfterMissedBlockAck",
                   "Specify whether explicit Block Ack Request should be sent upon missed Block Ack Response.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&QosTxop::m_useExplicitBarAfterMissedBlockAck),
                   MakeBooleanChecker ())
    .AddAttribute ("AddBaResponseTimeout",
                   "The timeout to wait for ADDBA response after the ACK to "
                   "ADDBA request is received.",
                   TimeValue (MilliSeconds (1)),
                   MakeTimeAccessor (&QosTxop::SetAddBaResponseTimeout,
                                     &QosTxop::GetAddBaResponseTimeout),
                   MakeTimeChecker ())
    .AddAttribute ("FailedAddBaTimeout",
                   "The timeout after a failed BA agreement. During this "
                   "timeout, the originator resumes sending packets using normal "
                   "MPDU. After that, BA agreement is reset and the originator "
                   "will retry BA negotiation.",
                   TimeValue (MilliSeconds (200)),
                   MakeTimeAccessor (&QosTxop::SetFailedAddBaTimeout,
                                     &QosTxop::GetFailedAddBaTimeout),
                   MakeTimeChecker ())
    .AddAttribute ("BlockAckManager",
                   "The BlockAckManager object.",
                   PointerValue (),
                   MakePointerAccessor (&QosTxop::m_baManager),
                   MakePointerChecker<BlockAckManager> ())
    .AddTraceSource ("TxopTrace",
                     "Trace source for txop start and duration times",
                     MakeTraceSourceAccessor (&QosTxop::m_txopTrace),
                     "ns3::TracedValueCallback::Time")
  ;
  return tid;
}

QosTxop::QosTxop ()
  : m_typeOfStation (STA),
    m_blockAckType (BlockAckType::COMPRESSED),
    m_startTxop (Seconds (0)),
    m_isAccessRequestedForRts (false),
    m_currentIsFragmented (false)
{
  NS_LOG_FUNCTION (this);
  m_qosBlockedDestinations = Create<QosBlockedDestinations> ();
  m_baManager = CreateObject<BlockAckManager> ();
  m_baManager->SetQueue (m_queue);
  m_baManager->SetBlockDestinationCallback (MakeCallback (&QosBlockedDestinations::Block, m_qosBlockedDestinations));
  m_baManager->SetUnblockDestinationCallback (MakeCallback (&QosBlockedDestinations::Unblock, m_qosBlockedDestinations));
  m_baManager->SetTxOkCallback (MakeCallback (&QosTxop::BaTxOk, this));
  m_baManager->SetTxFailedCallback (MakeCallback (&QosTxop::BaTxFailed, this));
}

QosTxop::~QosTxop ()
{
  NS_LOG_FUNCTION (this);
}

void
QosTxop::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_ackPolicySelector = 0;
  m_baManager = 0;
  m_qosBlockedDestinations = 0;
  Txop::DoDispose ();
}

Ptr<BlockAckManager>
QosTxop::GetBaManager (uint8_t tid) const
{
  return m_low->GetEdca (tid)->m_baManager;
}

bool
QosTxop::GetBaAgreementEstablished (Mac48Address address, uint8_t tid) const
{
  return m_baManager->ExistsAgreementInState (address, tid, OriginatorBlockAckAgreement::ESTABLISHED);
}

uint16_t
QosTxop::GetBaBufferSize (Mac48Address address, uint8_t tid) const
{
  return m_baManager->GetRecipientBufferSize (address, tid);
}

uint16_t
QosTxop::GetBaStartingSequence (Mac48Address address, uint8_t tid) const
{
  return m_baManager->GetOriginatorStartingSequence (address, tid);
}

void
QosTxop::RetransmitOutstandingMpdus (Mac48Address recipient, uint8_t tid)
{
  m_baManager->NotifyMissedBlockAck (recipient, tid);
}

Ptr<const WifiMacQueueItem>
QosTxop::PrepareBlockAckRequest (Mac48Address recipient, uint8_t tid) const
{
  NS_LOG_FUNCTION (this << recipient << +tid);

  CtrlBAckRequestHeader reqHdr = GetBaManager (tid)->GetBlockAckReqHeader (recipient, tid);
  Ptr<Packet> bar = Create<Packet> ();
  bar->AddHeader (reqHdr);

  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_CTL_BACKREQ);
  hdr.SetAddr1 (recipient);
  hdr.SetAddr2 (m_low->GetAddress ());
  hdr.SetAddr3 (m_low->GetBssid ());
  hdr.SetDsNotTo ();
  hdr.SetDsNotFrom ();
  hdr.SetNoRetry ();
  hdr.SetNoMoreFragments ();

  return Create<const WifiMacQueueItem> (bar, hdr);
}

Ptr<const WifiMacQueueItem>
QosTxop::PrepareMuBar (CtrlTriggerHeader trigger,
                       std::map<uint16_t, std::pair<Mac48Address, uint8_t>> recipients) const
{
  NS_LOG_FUNCTION (this << trigger);
  NS_ASSERT (trigger.GetNUserInfoFields () > 0 && !recipients.empty ());

  Mac48Address rxAddress;
  CtrlTriggerHeader muBar = trigger.GetCommonInfoField ();
  muBar.SetType (TriggerFrameType::MU_BAR_TRIGGER);

  // iterate over all the recipients
  for (auto& recipient : recipients)
    {
      auto userInfoIt = trigger.FindUserInfoWithAid (recipient.first);
      NS_ASSERT (userInfoIt != trigger.end ());
      CtrlTriggerUserInfoField& userInfo = muBar.AddUserInfoField (*userInfoIt);

      rxAddress = recipient.second.first;
      uint8_t tid = recipient.second.second;

      CtrlBAckRequestHeader reqHdr = GetBaManager (tid)->GetBlockAckReqHeader (rxAddress, tid);
      // Store the BAR in the Trigger Dependent User Info subfield
      userInfo.SetMuBarTriggerDepUserInfo (reqHdr);
    }

  Ptr<Packet> bar = Create<Packet> ();
  bar->AddHeader (muBar);
  // "If the Trigger frame has one User Info field and the AID12 subfield of the
  // User Info contains the AID of a STA, then the RA field is set to the address
  // of that STA". Otherwise, it is set to the broadcast address (Sec. 9.3.1.23 -
  // 802.11ax amendment draft 3.0)
  if (muBar.GetNUserInfoFields () > 1)
    {
      rxAddress = Mac48Address::GetBroadcast ();
    }

  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_CTL_TRIGGER);
  hdr.SetAddr1 (rxAddress);
  hdr.SetAddr2 (m_low->GetAddress ());
  hdr.SetAddr3 (m_low->GetBssid ());
  hdr.SetDsNotTo ();
  hdr.SetDsNotFrom ();
  hdr.SetNoRetry ();
  hdr.SetNoMoreFragments ();

  return Create<const WifiMacQueueItem> (bar, hdr);
}

void
QosTxop::ScheduleBar (Ptr<const WifiMacQueueItem> bar, bool skipIfNoData)
{
  m_baManager->ScheduleBar (bar, skipIfNoData);
}

void
QosTxop::SetWifiRemoteStationManager (const Ptr<WifiRemoteStationManager> remoteManager)
{
  Txop::SetWifiRemoteStationManager (remoteManager);
  NS_LOG_FUNCTION (this << remoteManager);
  m_baManager->SetWifiRemoteStationManager (m_stationManager);
}

void
QosTxop::SetAckPolicySelector (Ptr<WifiAckPolicySelector> ackSelector)
{
  NS_LOG_FUNCTION (this << ackSelector);
  m_ackPolicySelector = ackSelector;
}

Ptr<WifiAckPolicySelector>
QosTxop::GetAckPolicySelector (void) const
{
  return m_ackPolicySelector;
}

void
QosTxop::SetTypeOfStation (TypeOfStation type)
{
  NS_LOG_FUNCTION (this << +type);
  m_typeOfStation = type;
}

TypeOfStation
QosTxop::GetTypeOfStation (void) const
{
  return m_typeOfStation;
}

uint16_t
QosTxop::GetNextSequenceNumberFor (const WifiMacHeader *hdr)
{
  return m_txMiddle->GetNextSequenceNumberFor (hdr);
}

uint16_t
QosTxop::PeekNextSequenceNumberFor (const WifiMacHeader *hdr)
{
  return m_txMiddle->PeekNextSequenceNumberFor (hdr);
}

bool
QosTxop::IsQosOldPacket (Ptr<const WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << *mpdu);

  if (!mpdu->GetHeader ().IsQosData ())
    {
      return false;
    }

  Mac48Address recipient = mpdu->GetHeader ().GetAddr1 ();
  uint8_t tid = mpdu->GetHeader ().GetQosTid ();

  if (!GetBaAgreementEstablished (recipient, tid))
    {
      return false;
    }

  if (QosUtilsIsOldPacket (GetBaStartingSequence (recipient, tid),
                           mpdu->GetHeader ().GetSequenceNumber ()))
    {
      return true;
    }
  return false;
}

Ptr<const WifiMacQueueItem>
QosTxop::PeekNextFrame (uint8_t tid, Mac48Address recipient)
{
  NS_LOG_FUNCTION (this);
  WifiMacQueue::ConstIterator it = WifiMacQueue::EMPTY;

  // lambda to peek the next frame
  auto peek = [this, &tid, &recipient, &it] (Ptr<WifiMacQueue> queue)
    {
      if (tid == 8 && recipient.IsBroadcast ())  // undefined TID and recipient
        {
          return queue->PeekFirstAvailable (m_qosBlockedDestinations, it);
        }
      return queue->PeekByTidAndAddress (tid, recipient, it);
    };

  // check if there is a packet in the BlockAckManager retransmit queue
  it = peek (m_baManager->GetRetransmitQueue ());
  // remove old packets
  while (it != m_baManager->GetRetransmitQueue ()->end () && IsQosOldPacket (*it))
    {
      NS_LOG_DEBUG ("removing an old packet from BlockAckManager retransmit queue: " << **it);
      it = m_baManager->GetRetransmitQueue ()->Remove (it);
      it = peek (m_baManager->GetRetransmitQueue ());
    }
  if (it != m_baManager->GetRetransmitQueue ()->end ())
    {
      NS_LOG_DEBUG ("packet peeked from BlockAckManager retransmit queue: " << **it);
      return *it;
    }

  // otherwise, check if there is a packet in the EDCA queue
  it = WifiMacQueue::EMPTY;
  it = peek (m_queue);
  if (it != m_queue->end ())
    {
      // peek the next sequence number and check if it is within the transmit window
      // in case of QoS data frame
      uint16_t sequence = m_txMiddle->PeekNextSequenceNumberFor (&(*it)->GetHeader ());
      if ((*it)->GetHeader ().IsQosData ())
        {
          Mac48Address recipient = (*it)->GetHeader ().GetAddr1 ();
          uint8_t tid = (*it)->GetHeader ().GetQosTid ();

          if (GetBaAgreementEstablished (recipient, tid)
              && !IsInWindow (sequence, GetBaStartingSequence (recipient, tid), GetBaBufferSize (recipient, tid)))
            {
              NS_LOG_DEBUG ("packet beyond the end of the current transmit window");
              return 0;
            }
        }

      WifiMacHeader hdr = (*it)->GetHeader ();
      hdr.SetSequenceNumber (sequence);
      hdr.SetFragmentNumber (0);
      hdr.SetNoMoreFragments ();
      hdr.SetNoRetry ();
      Ptr<const WifiMacQueueItem> item = Create<const WifiMacQueueItem> ((*it)->GetPacket (), hdr, (*it)->GetTimeStamp ());
      NS_LOG_DEBUG ("packet peeked from EDCA queue: " << *item);
      return item;
    }

  return 0;
}

Ptr<WifiMacQueueItem>
QosTxop::DequeuePeekedFrame (Ptr<const WifiMacQueueItem> peekedItem, WifiTxVector txVector,
                             bool aggregate, uint32_t ampduSize, Time ppduDurationLimit)
{
  NS_LOG_FUNCTION (this << peekedItem << txVector << ampduSize << ppduDurationLimit);
  NS_ASSERT (peekedItem != 0);

  // do not dequeue the frame if it is a QoS data frame that does not meet the
  // max A-MPDU size limit (if applicable) or the duration limit (if applicable)
  if (peekedItem->GetHeader ().IsQosData () &&
      !m_low->IsWithinSizeAndTimeLimits (peekedItem, txVector, ampduSize, ppduDurationLimit))
    {
      return 0;
    }

  Mac48Address recipient = peekedItem->GetHeader ().GetAddr1 ();
  Ptr<WifiMacQueueItem> item;
  Ptr<const WifiMacQueueItem> testItem;
  WifiMacQueue::ConstIterator testIt;

  // the packet can only have been peeked from the Block Ack manager retransmit
  // queue if:
  // - the peeked packet is a QoS Data frame AND
  // - the peeked packet is not a broadcast frame AND
  // - an agreement has been established
  if (peekedItem->GetHeader ().IsQosData () && !recipient.IsBroadcast ()
      && GetBaAgreementEstablished (recipient, peekedItem->GetHeader ().GetQosTid ()))
    {
      uint8_t tid = peekedItem->GetHeader ().GetQosTid ();
      testIt = m_baManager->GetRetransmitQueue ()->PeekByTidAndAddress (tid, recipient);

      if (testIt != m_baManager->GetRetransmitQueue ()->end ())
        {
          testItem = *testIt;
          // if not null, the test packet must equal the peeked packet
          NS_ASSERT (testItem->GetPacket () == peekedItem->GetPacket ());
          // we should not be asked to dequeue an old packet
          NS_ASSERT (!QosUtilsIsOldPacket (GetBaStartingSequence (recipient, tid),
                                           testItem->GetHeader ().GetSequenceNumber ()));
          item = m_baManager->GetRetransmitQueue ()->Dequeue (testIt);
          NS_LOG_DEBUG ("dequeued from BA manager queue: " << *item);
          return item;
        }
    }

  // the packet has been peeked from the EDCA queue.
  uint16_t sequence = m_txMiddle->GetNextSequenceNumberFor (&peekedItem->GetHeader ());

  // If it is a QoS Data frame and it is not a broadcast frame, attempt A-MSDU
  // aggregation if aggregate is true
  if (peekedItem->GetHeader ().IsQosData ())
    {
      uint8_t tid = peekedItem->GetHeader ().GetQosTid ();
      testIt = m_queue->PeekByTidAndAddress (tid, recipient);

      NS_ASSERT (testIt != m_queue->end () && (*testIt)->GetPacket () == peekedItem->GetPacket ());

      // check if the peeked packet is within the transmit window
      if (GetBaAgreementEstablished (recipient, tid)
          && !IsInWindow (sequence, GetBaStartingSequence (recipient, tid), GetBaBufferSize (recipient, tid)))
        {
          NS_LOG_DEBUG ("packet beyond the end of the current transmit window");
          return 0;
        }

      // try A-MSDU aggregation
      if (m_low->GetMsduAggregator () != 0 && !recipient.IsBroadcast () && aggregate)
        {
          item = m_low->GetMsduAggregator ()->GetNextAmsdu (recipient, tid, txVector, ampduSize, ppduDurationLimit);
        }

      if (item != 0)
        {
          NS_LOG_DEBUG ("tx unicast A-MSDU");
        }
      else  // aggregation was not attempted or failed
        {
          item = m_queue->Dequeue (testIt);
        }
    }
  else
    {
      // the peeked packet is a non-QoS Data frame (e.g., a DELBA Request), hence
      // it was not peeked by TID, hence it must be the head of the queue
      item = m_queue->DequeueFirstAvailable (m_qosBlockedDestinations);
      NS_ASSERT (item != 0 && item->GetPacket () == peekedItem->GetPacket ());
    }

  // Assign a sequence number to the MSDU or A-MSDU dequeued from the EDCA queue
  NS_ASSERT (item != 0);
  item->GetHeader ().SetSequenceNumber (sequence);
  item->GetHeader ().SetFragmentNumber (0);
  item->GetHeader ().SetNoMoreFragments ();
  item->GetHeader ().SetNoRetry ();
  NS_LOG_DEBUG ("dequeued from EDCA queue: " << *item);

  return item;
}

MacLowTransmissionParameters
QosTxop::GetTransmissionParameters (Ptr<const WifiMacQueueItem> frame) const
{
  NS_LOG_FUNCTION (this << *frame);

  MacLowTransmissionParameters params;
  Mac48Address recipient = frame->GetHeader ().GetAddr1 ();

  params.DisableNextData ();

  // broadcast frames (except MU-BAR)
  if (recipient.IsBroadcast () && !frame->GetHeader ().IsTrigger ())
    {
      params.DisableRts ();
      params.DisableAck ();
      return params;
    }
  if (frame->GetHeader ().IsMgt ())
    {
      params.DisableRts ();
      params.EnableAck ();
      return params;
    }

  // Enable/disable RTS
  if (!frame->GetHeader ().IsBlockAckReq ()
      && !frame->GetHeader ().IsTrigger ()
      && m_stationManager->NeedRts (recipient, &frame->GetHeader (),
                                    frame->GetPacket (), m_low->GetDataTxVector (frame))
      && !m_low->IsCfPeriod ())
    {
      params.EnableRts ();
    }
  else
    {
      params.DisableRts ();
    }

  CtrlTriggerHeader trigger;
  if (frame->GetHeader ().IsTrigger ())
    {
      frame->GetPacket ()->PeekHeader (trigger);
    }

  // Select ack technique.
  if (frame->GetHeader ().IsQosData ())
    {
      // Assume normal ack by default
      params.EnableAck ();
    }
  else if (frame->GetHeader ().IsBlockAckReq ())
    {
      CtrlBAckRequestHeader baReqHdr;
      frame->GetPacket ()->PeekHeader (baReqHdr);
      uint8_t tid = baReqHdr.GetTidInfo ();
      params.EnableBlockAck (GetBlockAckType (recipient, tid));
    }
  else if (frame->GetHeader ().IsTrigger () && trigger.IsMuBar ())
    {
      Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_low->GetPhy ()->GetDevice ());
      NS_ASSERT (dev != 0);
      Ptr<ApWifiMac> apMac = DynamicCast<ApWifiMac> (dev->GetMac ());
      NS_ASSERT_MSG (apMac, "Only APs can send MU-BAR Trigger frames");
      const std::map<uint16_t, Mac48Address>& aidAddressMap = apMac->GetStaList ();

      for (auto& userInfo : trigger)
        {
          auto addressIt = aidAddressMap.find (userInfo.GetAid12 ());
          NS_ASSERT_MSG (addressIt != aidAddressMap.end (), "Sending MU-BAR to unassociated station");
          params.EnableBlockAck (addressIt->second,
                                 GetBlockAckType (addressIt->second,
                                                  userInfo.GetMuBarTriggerDepUserInfo ().GetTidInfo ()));
        }
    }

  return params;
}

BlockAckReqType
QosTxop::GetBlockAckReqType (Mac48Address recipient, uint8_t tid) const
{
  return GetBaManager (tid)->GetBlockAckReqType (recipient, tid);
}

BlockAckType
QosTxop::GetBlockAckType (Mac48Address recipient, uint8_t tid) const
{
  return GetBaManager (tid)->GetBlockAckType (recipient, tid);
}

void
QosTxop::UpdateCurrentPacket (Ptr<WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << *mpdu);
  m_currentMpdu = mpdu;
}

Ptr<const WifiMacQueueItem>
QosTxop::GetBar (bool remove, uint8_t tid, Mac48Address address)
{
  return m_baManager->GetBar (remove, tid, address);
}

uint32_t
QosTxop::GetBufferSize (Mac48Address address, uint8_t tid) const
{
  return m_queue->GetNBytes (tid, address) +
         m_baManager->GetRetransmitQueue ()->GetNBytes (tid, address);
}

void
QosTxop::NotifyAccessGranted (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_accessRequested);
  m_accessRequested = false;
  m_isAccessRequestedForRts = false;
  m_startTxop = Simulator::Now ();
  // discard the current packet if it is a QoS Data frame with expired lifetime
  if (m_currentMpdu != 0 && m_currentMpdu->GetHeader ().IsQosData ()
      && (m_currentMpdu->GetTimeStamp () + m_queue->GetMaxDelay () < Simulator::Now ()))
    {
      NS_LOG_DEBUG ("the lifetime of current packet expired");
      m_currentMpdu = 0;
    }
  // If the current packet is a QoS Data frame, then there must be no block ack agreement
  // established with the receiver for the TID of the packet. Indeed, retransmission
  // of MPDUs sent under a block ack agreement is handled through the retransmit queue.
  NS_ASSERT (m_currentMpdu == 0 || !m_currentMpdu->GetHeader ().IsQosData ()
             || !GetBaAgreementEstablished (m_currentMpdu->GetHeader ().GetAddr1 (),
                                            m_currentMpdu->GetHeader ().GetQosTid ()));

  if (m_currentMpdu == 0)
    {
      Ptr<const WifiMacQueueItem> peekedItem = m_baManager->GetBar ();
      if (peekedItem != 0)
        {
          m_currentMpdu = Copy (peekedItem);
        }
      else
        {
          peekedItem = PeekNextFrame ();
          if (peekedItem == 0)
            {
              NS_LOG_DEBUG ("no packets available for transmission");
              return;
            }
          // check if a Block Ack agreement needs to be established
          m_currentMpdu = Copy (peekedItem);
          if (m_currentMpdu->GetHeader ().IsQosData () && !m_currentMpdu->GetHeader ().GetAddr1 ().IsBroadcast ()
              && m_stationManager->GetQosSupported (m_currentMpdu->GetHeader ().GetAddr1 ())
              && (!m_baManager->ExistsAgreement (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                 m_currentMpdu->GetHeader ().GetQosTid ())
                  || m_baManager->ExistsAgreementInState (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                          m_currentMpdu->GetHeader ().GetQosTid (), OriginatorBlockAckAgreement::RESET))
              && SetupBlockAckIfNeeded ())
            {
              return;
            }

          m_stationManager->UpdateFragmentationThreshold ();
          Ptr<WifiMacQueueItem> item;
          // non-broadcast QoS data frames may be sent in MU PPDUs. Given that at this stage
          // we do not know the bandwidth it would be given nor the selected acknowledgment
          // sequence, we cannot determine the constraints on size and duration limit. Hence,
          // we only peek the non-broadcast QoS data frame. MacLow will be in charge of
          // computing the correct limits and dequeue the frame.
          if (peekedItem->GetHeader ().IsQosData () && !peekedItem->GetHeader ().GetAddr1 ().IsBroadcast ()
              && !NeedFragmentation ())
            {
              item = Copy (peekedItem);
            }
          else
            {
              // compute the limit on the PPDU duration due to the TXOP duration, if any
              Time ppduDurationLimit = Seconds (0);
              if (peekedItem->GetHeader ().IsQosData () && GetTxopLimit ().IsStrictlyPositive ())
                {
                  MacLowTransmissionParameters params = GetTransmissionParameters (peekedItem);
                  ppduDurationLimit = GetTxopRemaining () - m_low->CalculateOverheadTxTime (peekedItem, params);
                }

                // dequeue the peeked item if it fits within the TXOP duration, if any
                item = DequeuePeekedFrame (peekedItem, m_low->GetDataTxVector (peekedItem),
                                          !NeedFragmentation (), 0, ppduDurationLimit);
            }

          if (item == 0)
            {
              NS_LOG_DEBUG ("Not enough time in the current TXOP");
              return;
            }
          m_currentMpdu = item;

          m_fragmentNumber = 0;
        }
      NS_ASSERT (m_currentMpdu != 0);
    }

  m_currentParams = GetTransmissionParameters (m_currentMpdu);

  if (m_currentMpdu->GetHeader ().GetAddr1 ().IsGroup ())
    {
      NS_LOG_DEBUG ("tx broadcast");
      m_low->StartTransmission (m_currentMpdu, m_currentParams, this);
    }
  //With COMPRESSED_BLOCK_ACK fragmentation must be avoided.
  else if (((m_currentMpdu->GetHeader ().IsQosData () && !m_currentMpdu->GetHeader ().IsQosAmsdu ())
            || (m_currentMpdu->GetHeader ().IsData () && !m_currentMpdu->GetHeader ().IsQosData ()))
           && (GetBlockAckThreshold () == 0 || m_blockAckType.m_variant == BlockAckType::BASIC)
           && NeedFragmentation ())
    {
      m_currentIsFragmented = true;
      m_currentParams.DisableRts ();
      WifiMacHeader hdr;
      Ptr<Packet> fragment = GetFragmentPacket (&hdr);
      if (IsLastFragment ())
        {
          NS_LOG_DEBUG ("fragmenting last fragment size=" << fragment->GetSize ());
          m_currentParams.DisableNextData ();
        }
      else
        {
          NS_LOG_DEBUG ("fragmenting size=" << fragment->GetSize ());
          m_currentParams.EnableNextData (GetNextFragmentSize ());
        }
      m_low->StartTransmission (Create<WifiMacQueueItem> (fragment, hdr),
                                m_currentParams, this);
    }
  else
    {
      m_currentIsFragmented = false;
      m_low->StartTransmission (m_currentMpdu, m_currentParams, this);
    }
}

void QosTxop::NotifyInternalCollision (void)
{
  NS_LOG_FUNCTION (this);
  bool resetDcf = false;
  // If an internal collision is experienced, the frame involved may still
  // be sitting in the queue, and m_currentPacket may still be null.
  Ptr<WifiMacQueueItem> mpdu;
  if (m_currentMpdu == 0)
    {
      Ptr<const WifiMacQueueItem> item = PeekNextFrame ();
      if (item)
        {
          mpdu = Copy (item);
        }
    }
  else
    {
      mpdu = m_currentMpdu;
    }
  if (mpdu != 0)
    {
      if (m_isAccessRequestedForRts)
        {
          if (!NeedRtsRetransmission (mpdu))
            {
              resetDcf = true;
              m_stationManager->ReportFinalRtsFailed (mpdu->GetHeader ().GetAddr1 (), &mpdu->GetHeader ());
            }
          else
            {
              m_stationManager->ReportRtsFailed (mpdu->GetHeader ().GetAddr1 (), &mpdu->GetHeader ());
            }
        }
      else if (mpdu->GetHeader ().GetAddr1 () == Mac48Address::GetBroadcast ())
        {
          resetDcf = false;
        }
      else
        {
          if (!NeedDataRetransmission (mpdu))
            {
              resetDcf = true;
              m_stationManager->ReportFinalDataFailed (mpdu->GetHeader ().GetAddr1 (), &mpdu->GetHeader (),
                                                       mpdu->GetPacket ()->GetSize ());
            }
          else
            {
              m_stationManager->ReportDataFailed (mpdu->GetHeader ().GetAddr1 (), &mpdu->GetHeader (),
                                                  mpdu->GetPacket ()->GetSize ());
            }
        }
      if (resetDcf)
        {
          NS_LOG_DEBUG ("reset DCF");
          if (!m_txFailedCallback.IsNull ())
            {
              m_txFailedCallback (mpdu->GetHeader ());
            }
          //to reset the dcf.
          if (m_currentMpdu)
            {
              NS_LOG_DEBUG ("Discarding m_currentPacket");
              m_currentMpdu = 0;
            }
          else
            {
              NS_LOG_DEBUG ("Dequeueing and discarding head of queue");
              m_queue->Remove ();
            }
          ResetCw ();
        }
      else
        {
          UpdateFailedCw ();
        }
    }
  m_backoff = m_rng->GetInteger (0, GetCw ());
  m_backoffTrace (m_backoff);
  StartBackoffNow (m_backoff);
  RestartAccessIfNeeded ();
}

void
QosTxop::NotifyCollision (void)
{
  NS_LOG_FUNCTION (this);
  m_backoff = m_rng->GetInteger (0, GetCw ());
  m_backoffTrace (m_backoff);
  StartBackoffNow (m_backoff);
  RestartAccessIfNeeded ();
}

void
QosTxop::NotifyMissedCts (std::list<Ptr<WifiMacQueueItem>> mpduList)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("missed cts");
  NS_ASSERT (!mpduList.empty ());
  if (!NeedRtsRetransmission (m_currentMpdu))
    {
      NS_LOG_DEBUG ("Cts Fail");
      m_stationManager->ReportFinalRtsFailed (m_currentMpdu->GetHeader ().GetAddr1 (), &m_currentMpdu->GetHeader ());
      if (!m_txFailedCallback.IsNull ())
        {
          m_txFailedCallback (m_currentMpdu->GetHeader ());
        }
      for (auto& mpdu : mpduList)
        {
          m_baManager->NotifyDiscardedMpdu (mpdu);
        }
      //to reset the dcf.
      m_currentMpdu = 0;
      ResetCw ();
      m_cwTrace = GetCw ();
    }
  else
    {
      UpdateFailedCw ();
      m_cwTrace = GetCw ();
      // if a BA agreement is established, store the MPDUs in the block ack manager
      // retransmission queue. Otherwise, this QosTxop will handle the retransmission
      // of the (single) frame
      if (mpduList.size () > 1 ||
          (mpduList.front ()->GetHeader ().IsQosData ()
           && GetBaAgreementEstablished (mpduList.front ()->GetHeader ().GetAddr1 (),
                                         mpduList.front ()->GetHeader ().GetQosTid ())))
        {
          for (auto it = mpduList.rbegin (); it != mpduList.rend (); it++)
            {
              m_baManager->GetRetransmitQueue ()->PushFront (*it);
            }
          m_currentMpdu = 0;
        }
    }
  m_backoff = m_rng->GetInteger (0, GetCw ());
  m_backoffTrace (m_backoff);
  StartBackoffNow (m_backoff);
  RestartAccessIfNeeded ();
}

void
QosTxop::GotAck (void)
{
  NS_LOG_FUNCTION (this);
  if (!m_currentIsFragmented
      || !m_currentParams.HasNextPacket ()
      || m_currentMpdu->GetHeader ().IsQosAmsdu ())
    {
      NS_LOG_DEBUG ("got ack. tx done.");
      if (!m_txOkCallback.IsNull ())
        {
          m_txOkCallback (m_currentMpdu->GetHeader ());
        }

      if (m_currentMpdu->GetHeader ().IsAction ())
        {
          WifiActionHeader actionHdr;
          Ptr<Packet> p = m_currentMpdu->GetPacket ()->Copy ();
          p->RemoveHeader (actionHdr);
          if (actionHdr.GetCategory () == WifiActionHeader::BLOCK_ACK)
            {
              if (actionHdr.GetAction ().blockAck == WifiActionHeader::BLOCK_ACK_DELBA)
                {
                  MgtDelBaHeader delBa;
                  p->PeekHeader (delBa);
                  if (delBa.IsByOriginator ())
                    {
                      m_baManager->DestroyAgreement (m_currentMpdu->GetHeader ().GetAddr1 (), delBa.GetTid ());
                    }
                  else
                    {
                      m_low->DestroyBlockAckAgreement (m_currentMpdu->GetHeader ().GetAddr1 (), delBa.GetTid ());
                    }
                }
              else if (actionHdr.GetAction ().blockAck == WifiActionHeader::BLOCK_ACK_ADDBA_REQUEST)
                {
                  // Setup addba response timeout
                  MgtAddBaRequestHeader addBa;
                  p->PeekHeader (addBa);
                  Simulator::Schedule (m_addBaResponseTimeout,
                                       &QosTxop::AddBaResponseTimeout, this,
                                       m_currentMpdu->GetHeader ().GetAddr1 (), addBa.GetTid ());
                }
            }
        }
      if (m_currentMpdu->GetHeader ().IsQosData ()
          && GetBaAgreementEstablished (m_currentMpdu->GetHeader ().GetAddr1 (),
                                        m_currentMpdu->GetHeader ().GetQosTid ()))
        {
          // notify the BA manager that the current packet was acknowledged
          m_baManager->NotifyGotAck (m_currentMpdu);
        }
      m_currentMpdu = 0;
      ResetCw ();
    }
  else
    {
      NS_LOG_DEBUG ("got ack. tx not done, size=" << m_currentMpdu->GetPacket ()->GetSize ());
    }
}

void
QosTxop::MissedAck (bool txSuccess)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("missed ack");
  bool resetCw = txSuccess;  // Reset CW if the tx succeeded or we give up retransmitting

  if (!NeedDataRetransmission (m_currentMpdu))
    {
      NS_LOG_DEBUG ("Ack Fail");
      m_stationManager->ReportFinalDataFailed (m_currentMpdu->GetHeader ().GetAddr1 (), &m_currentMpdu->GetHeader (),
                                               m_currentMpdu->GetPacket ()->GetSize ());
      if (!m_txFailedCallback.IsNull ())
        {
          m_txFailedCallback (m_currentMpdu->GetHeader ());
        }
      if (m_currentMpdu->GetHeader ().IsAction ())
        {
          WifiActionHeader actionHdr;
          m_currentMpdu->GetPacket ()->PeekHeader (actionHdr);
          if (actionHdr.GetCategory () == WifiActionHeader::BLOCK_ACK)
            {
              uint8_t tid = GetTid (m_currentMpdu->GetPacket (), m_currentMpdu->GetHeader ());
              if (m_baManager->ExistsAgreementInState (m_currentMpdu->GetHeader ().GetAddr1 (), tid, OriginatorBlockAckAgreement::PENDING))
                {
                  NS_LOG_DEBUG ("No ACK after ADDBA request");
                  m_baManager->NotifyAgreementNoReply (m_currentMpdu->GetHeader ().GetAddr1 (), tid);
                  Simulator::Schedule (m_failedAddBaTimeout, &QosTxop::ResetBa, this,
                                       m_currentMpdu->GetHeader ().GetAddr1 (), tid);
                }
            }
        }
      if (m_currentMpdu->GetHeader ().IsQosData ())
        {
          GetBaManager (m_currentMpdu->GetHeader ().GetQosTid ())->NotifyDiscardedMpdu (m_currentMpdu);
        }
      m_currentMpdu = 0;
      resetCw = true;
    }
  else
    {
      NS_LOG_DEBUG ("Retransmit");
      m_stationManager->ReportDataFailed (m_currentMpdu->GetHeader ().GetAddr1 (), &m_currentMpdu->GetHeader (),
                                          m_currentMpdu->GetPacket ()->GetSize ());
      m_currentMpdu->GetHeader ().SetRetry ();
      if (m_currentMpdu->GetHeader ().IsQosData ())
        {
          uint8_t tid = m_currentMpdu->GetHeader ().GetQosTid ();
          if (m_low->GetEdca (tid)->GetBaAgreementEstablished (m_currentMpdu->GetHeader ().GetAddr1 (), tid))
            {
              // notify the BA manager that the current packet was not acknowledged
              GetBaManager (tid)->NotifyMissedAck (m_currentMpdu);
              // let the BA manager handle its retransmission
              m_currentMpdu = 0;
            }
        }
    }
  if (resetCw)
    {
      ResetCw ();
    }
  else
    {
      UpdateFailedCw ();
    }
  // start next packet if transmission succeeded and TXOP remains
  if (txSuccess && GetTxopLimit ().IsStrictlyPositive () && GetTxopRemaining () > m_low->GetSifs ())
    {
      if (m_stationManager->GetRifsPermitted ())
        {
          Simulator::Schedule (m_low->GetRifs (), &QosTxop::StartNextPacket, this);
        }
      else
        {
          Simulator::Schedule (m_low->GetSifs (), &QosTxop::StartNextPacket, this);
        }
    }
  else
    {
      TerminateTxop ();
    }
}

void
QosTxop::MissedBlockAck (std::map <uint16_t, Ptr<WifiPsdu>> psduMap, const MacLowTransmissionParameters& params,
                         bool txSuccess)
{
  NS_LOG_FUNCTION (this << params << txSuccess);
  /*
   * If the BlockAck frame is lost, the originator may transmit a BlockAckReq
   * frame to solicit an immediate BlockAck frame or it may retransmit the Data
   * frames. (IEEE std 802.11-2016 sec. 10.24.7.7)
   */
  bool resetCw = txSuccess;  // Reset CW if the tx succeeded or we give up retransmitting

  /* One or more Block Acks were not received in response to a MU-BAR sent as SU PPDU */
  if (psduMap.size () == 1 && psduMap.begin ()->second->GetHeader (0).IsTrigger ())
    {
      CtrlTriggerHeader trigger;
      psduMap.begin ()->second->GetPayload (0)->PeekHeader (trigger);
      NS_ASSERT (trigger.IsMuBar ());

      // Find the stations which did not respond with a Block Ack and to which
      // a BAR needs to be retransmitted
      std::map<uint16_t, std::pair<Mac48Address, uint8_t>> recipients;

      for (auto& sta : params.GetStationsReceiveBlockAckFrom ())
        {
          uint16_t staId = m_low->GetStaId (sta);
          auto userInfoIt = trigger.FindUserInfoWithAid (staId);
          NS_ASSERT (userInfoIt != trigger.end ());
          CtrlBAckRequestHeader blockAckReq = userInfoIt->GetMuBarTriggerDepUserInfo ();
          uint8_t tid = blockAckReq.GetTidInfo ();

          if (GetBaManager (tid)->NeedBarRetransmission (tid, sta))
            {
              recipients[staId] = std::make_pair (sta, tid);
            }
          else
            {
              NS_LOG_DEBUG ("Block Ack Request to " << sta << " failed");
              GetBaManager (tid)->DiscardOutstandingMpdus (sta, tid);
            }
        }

      if (recipients.empty ())
        {
          // no BAR needs to be retransmitted, hence reset the contention window
          NS_LOG_DEBUG ("No BAR/MU-BAR needs to be retransmitted");
          resetCw = true;
        }
      else if (recipients.size () == 1)
        {
          // the Block Ack response is missing from one station only, then
          // schedule a Block Ack Request
          NS_LOG_DEBUG ("Transmit block ack request to " << recipients.begin ()->second.first);
          ScheduleBar (PrepareBlockAckRequest (recipients.begin ()->second.first,
                                               recipients.begin ()->second.second));
        }
      else
        {
          // Schedule a new MU-BAR only containing a request for the stations from
          // which a Block Ack has not been received. Such stations are those listed
          // in the TX params.
          NS_LOG_DEBUG ("Transmit a MU-BAR Trigger Frame");
          ScheduleBar (PrepareMuBar (trigger, recipients));
        }
    }
  else
    {
      bool giveUpRetransmittingAll = true;

      for (auto& psdu : psduMap)
        {
          UpdateCurrentPacket (*psdu.second->begin ());
          uint8_t tid = GetTid (m_currentMpdu->GetPacket (), m_currentMpdu->GetHeader ());
          if (GetAmpduExist (m_currentMpdu->GetHeader ().GetAddr1 ()))
            {
              m_stationManager->ReportAmpduTxStatus (m_currentMpdu->GetHeader ().GetAddr1 (), tid, 0,
                                                     psdu.second->GetNMpdus (), 0, 0);
            }

          if (m_useExplicitBarAfterMissedBlockAck || m_currentMpdu->GetHeader ().IsBlockAckReq ())
            {
              if (NeedBarRetransmission ())
                {
                  NS_LOG_DEBUG ("Retransmit block ack request");
                  giveUpRetransmittingAll = false;
                  if (m_currentMpdu->GetHeader ().IsBlockAckReq ())
                    {
                      m_currentMpdu->GetHeader ().SetRetry ();
                      ScheduleBar (m_currentMpdu);
                    }
                  else // missed block ack after data frame with Implicit BAR Ack policy
                    {
                      ScheduleBar (PrepareBlockAckRequest (m_currentMpdu->GetHeader ().GetAddr1 (), tid));
                    }
                }
              else
                {
                  NS_LOG_DEBUG ("Block Ack Request Fail");
                  // if a BA agreement exists, we can get here if there is no outstanding
                  // MPDU that has not expired yet.
                  if (GetBaManager (tid)->ExistsAgreementInState (m_currentMpdu->GetHeader ().GetAddr1 (), tid,
                                                                  OriginatorBlockAckAgreement::ESTABLISHED))
                    {
                      // If there is any (expired) outstanding MPDU, request the BA manager to discard
                      // it, which involves the scheduling of a BAR to advance the recipient's window
                      if (GetBaManager (tid)->GetNBufferedPackets (m_currentMpdu->GetHeader ().GetAddr1 (), tid) > 0)
                        {
                          GetBaManager (tid)->DiscardOutstandingMpdus (m_currentMpdu->GetHeader ().GetAddr1 (), tid);
                        }
                      // otherwise, it means that we have not received a Block Ack in response to a
                      // Block Ack Request sent while no frame was outstanding, whose purpose was therefore
                      // to advance the recipient's window. If there is any (unexpired) data frame queued,
                      // keep transmitting a Block Ack Request to advance the recipient's window.
                      else if (m_low->GetEdca (tid)->PeekNextFrame (tid, m_currentMpdu->GetHeader ().GetAddr1 ()) != 0)
                        {
                          ScheduleBar (PrepareBlockAckRequest (m_currentMpdu->GetHeader ().GetAddr1 (), tid));
                        }
                      else
                        {
                          // No data frame is queued. Record that a Block Ack Request must be sent before
                          // any data that may arrive later.
                          ScheduleBar (PrepareBlockAckRequest (m_currentMpdu->GetHeader ().GetAddr1 (), tid), true);
                        }
                    }
                }
            }
          else
            {
              // implicit BAR and do not use BAR after missed block ack, hence try to retransmit data frames
              if (!NeedDataRetransmission (m_currentMpdu))
                {
                  NS_LOG_DEBUG ("Block Ack Fail");
                  if (!m_txFailedCallback.IsNull ())
                    {
                      m_txFailedCallback (m_currentMpdu->GetHeader ());
                    }
                  GetBaManager (tid)->DiscardOutstandingMpdus (m_currentMpdu->GetHeader ().GetAddr1 (), tid);
                }
              else
                {
                  NS_LOG_DEBUG ("Retransmit");
                  giveUpRetransmittingAll = false;
                  GetBaManager (tid)->NotifyMissedBlockAck (m_currentMpdu->GetHeader ().GetAddr1 (), tid);
                }
            }
        }
      if (giveUpRetransmittingAll)
        {
          // no retransmission is needed, hence reset the contention window
          NS_LOG_DEBUG ("No BAR/QoS data frame needs to be retransmitted");
          resetCw = true;
        }
    }

  m_currentMpdu = 0;
  if (!params.HasDlMuAckSequence () && psduMap.size () == 1 && psduMap.begin ()->first != SU_STA_ID)
    {
      // This is a PSDU sent in an HE TB PPDU. An HE STA resumes the EDCA backoff procedure
      // without modifying CW or the backoff counter for the associated EDCAF, after
      // transmission of an MPDU in an HE TB PPDU regardless of whether the STA has received
      // the corresponding acknowledgment frame in response to the MPDU sent in the HE TB PPDU
      // (Sec. 10.22.2.2 of 11ax Draft 3.0)
      return;
    }

  if (resetCw)
    {
      ResetCw ();
    }
  else
    {
      UpdateFailedCw ();
    }

  // start next packet if transmission succeeded and TXOP remains
  if (txSuccess && GetTxopLimit ().IsStrictlyPositive () && GetTxopRemaining () > m_low->GetSifs ())
    {
      if (m_stationManager->GetRifsPermitted ())
        {
          Simulator::Schedule (m_low->GetRifs (), &QosTxop::StartNextPacket, this);
        }
      else
        {
          Simulator::Schedule (m_low->GetSifs (), &QosTxop::StartNextPacket, this);
        }
    }
  else
    {
      TerminateTxop ();
    }
}

void
QosTxop::RestartAccessIfNeeded (void)
{
  NS_LOG_FUNCTION (this);
  if ((m_currentMpdu != 0
       // check first if the BA manager retransmit queue is empty, so that expired
       // frames (if any) are removed and a Block Ack Request is scheduled to advance
       // the starting sequence number of the transmit (and receiver) window
       || m_baManager->HasPackets () || !m_queue->IsEmpty ())
      && !IsAccessRequested ())
    {
      Ptr<WifiMacQueueItem> mpdu;
      if (m_currentMpdu != 0)
        {
          mpdu = m_currentMpdu;
        }
      else
        {
          Ptr<const WifiMacQueueItem> item = PeekNextFrame ();
          if (item)
            {
              mpdu = Copy (item);
            }
        }
      if (mpdu != 0)
        {
          m_isAccessRequestedForRts = m_stationManager->NeedRts (mpdu->GetHeader ().GetAddr1 (), &mpdu->GetHeader (),
                                                                 mpdu->GetPacket (), m_low->GetDataTxVector (mpdu));
        }
      else
        {
          m_isAccessRequestedForRts = false;
        }
      m_channelAccessManager->RequestAccess (this);
    }
}

void
QosTxop::StartAccessIfNeeded (void)
{
  NS_LOG_FUNCTION (this);
  if (m_currentMpdu == 0
      // check first if the BA manager retransmit queue is empty, so that expired
      // frames (if any) are removed and a Block Ack Request is scheduled to advance
      // the starting sequence number of the transmit (and receiver) window
      && (m_baManager->HasPackets () || !m_queue->IsEmpty ())
      && !IsAccessRequested ())
    {
      Ptr<const WifiMacQueueItem> item = PeekNextFrame ();
      if (item != 0)
        {
          m_isAccessRequestedForRts = m_stationManager->NeedRts (item->GetHeader ().GetAddr1 (), &item->GetHeader (),
                                                                 item->GetPacket (), m_low->GetDataTxVector (item));
        }
      else
        {
          m_isAccessRequestedForRts = false;
        }
      m_channelAccessManager->RequestAccess (this);
    }
}

bool
QosTxop::NeedBarRetransmission (void)
{
  uint8_t tid = 0;
  if (m_currentMpdu->GetHeader ().IsQosData ())
    {
      tid = m_currentMpdu->GetHeader ().GetQosTid ();
    }
  else if (m_currentMpdu->GetHeader ().IsBlockAckReq ())
    {
      CtrlBAckRequestHeader baReqHdr;
      m_currentMpdu->GetPacket ()->PeekHeader (baReqHdr);
      tid = baReqHdr.GetTidInfo ();
    }
  else if (m_currentMpdu->GetHeader ().IsBlockAck ())
    {
      CtrlBAckResponseHeader baRespHdr;
      m_currentMpdu->GetPacket ()->PeekHeader (baRespHdr);
      tid = baRespHdr.GetTidInfo ();
    }
  return GetBaManager (tid)->NeedBarRetransmission (tid, m_currentMpdu->GetHeader ().GetAddr1 ());
}

void
QosTxop::StartNextPacket (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (GetTxopLimit ().IsStrictlyPositive () && GetTxopRemaining ().IsStrictlyPositive ());

  m_currentMpdu = 0;
  // peek the next BlockAckReq, if any
  Ptr<const WifiMacQueueItem> nextFrame = m_baManager->GetBar (false);
  bool isPeekedFromBaManager = true;

  if (nextFrame == 0)
    {
      nextFrame = PeekNextFrame ();
      isPeekedFromBaManager = false;
    }

  if (nextFrame != 0)
    {
      MacLowTransmissionParameters params = GetTransmissionParameters (nextFrame);

      if (GetTxopRemaining () >= m_low->CalculateOverallTxTime (nextFrame, params))
        {
          m_currentMpdu = Copy (nextFrame);

          // check if a Block Ack agreement needs to be established
          if (m_currentMpdu->GetHeader ().IsQosData () && !m_currentMpdu->GetHeader ().GetAddr1 ().IsBroadcast ()
              && m_stationManager->GetQosSupported (m_currentMpdu->GetHeader ().GetAddr1 ())
              && (!m_baManager->ExistsAgreement (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                 m_currentMpdu->GetHeader ().GetQosTid ())
                  || m_baManager->ExistsAgreementInState (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                          m_currentMpdu->GetHeader ().GetQosTid (),
                                                          OriginatorBlockAckAgreement::RESET))
              && SetupBlockAckIfNeeded ())
            {
              return;
            }

          if (isPeekedFromBaManager)
            {
              m_currentMpdu = Copy (m_baManager->GetBar ());
            }
          // non-broadcast QoS data frames may be sent in MU PPDUs. Given that at this stage
          // we do not know the bandwidth it would be given nor the selected acknowledgment
          // sequence, we cannot determine the constraints on size and duration limit. Hence,
          // we only peek the non-broadcast QoS data frame. MacLow will be in charge of
          // computing the correct limits and dequeue the frame.
          else if (!nextFrame->GetHeader ().IsQosData () || nextFrame->GetHeader ().GetAddr1 ().IsBroadcast ())
            {
              // dequeue the peeked frame
              m_currentMpdu = DequeuePeekedFrame (nextFrame, m_low->GetDataTxVector (nextFrame));
            }

          NS_ASSERT (m_currentMpdu != 0);
          NS_LOG_DEBUG ("start next packet " << *m_currentMpdu << " within the current TXOP");

          m_currentParams = params;
          m_stationManager->UpdateFragmentationThreshold ();
          m_fragmentNumber = 0;
          GetLow ()->StartTransmission (m_currentMpdu, m_currentParams, this);
          return;
        }
    }

  // terminate TXOP because no (suitable) frame was found
  TerminateTxop ();
}

void
QosTxop::TerminateTxop (void)
{
  NS_LOG_FUNCTION (this);
  if (m_startTxop.IsStrictlyPositive () && GetTxopLimit ().IsStrictlyPositive ())
    {
      NS_LOG_DEBUG ("Terminating TXOP. Duration = " << Simulator::Now () - m_startTxop);
      m_txopTrace (m_startTxop, Simulator::Now () - m_startTxop);
      m_startTxop = Seconds (0);
    }
  m_cwTrace = GetCw ();
  m_backoff = m_rng->GetInteger (0, GetCw ());
  m_backoffTrace (m_backoff);
  StartBackoffNow (m_backoff);
  RestartAccessIfNeeded ();
}

Time
QosTxop::GetTxopRemaining (void) const
{
  Time remainingTxop = GetTxopLimit ();
  remainingTxop -= (Simulator::Now () - m_startTxop);
  if (remainingTxop.IsStrictlyNegative ())
    {
      remainingTxop = Seconds (0);
    }
  NS_LOG_FUNCTION (this << remainingTxop);
  return remainingTxop;
}

void
QosTxop::EndTxNoAck (void)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("a transmission that did not require an ACK just finished");
  m_currentMpdu = 0;
  ResetCw ();
  TerminateTxop ();
}

bool
QosTxop::NeedFragmentation (void) const
{
  NS_LOG_FUNCTION (this);
  if (m_stationManager->GetVhtSupported ()
      || m_stationManager->GetHeSupported ()
      || GetAmpduExist (m_currentMpdu->GetHeader ().GetAddr1 ())
      || (m_stationManager->GetHtSupported ()
          && m_currentMpdu->GetHeader ().IsQosData ()
          && GetBaAgreementEstablished (m_currentMpdu->GetHeader ().GetAddr1 (),
                                        GetTid (m_currentMpdu->GetPacket (), m_currentMpdu->GetHeader ()))
          && GetLow ()->GetMpduAggregator () != 0
          && GetLow ()->GetMpduAggregator ()->GetMaxAmpduSize (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                               GetTid (m_currentMpdu->GetPacket (),
                                                                       m_currentMpdu->GetHeader ()),
                                                               WIFI_MOD_CLASS_HT) >= m_currentMpdu->GetPacket ()->GetSize ()))
    {
      //MSDU is not fragmented when it is transmitted using an HT-immediate or
      //HT-delayed Block Ack agreement or when it is carried in an A-MPDU.
      return false;
    }
  bool needTxopFragmentation = false;
  if (GetTxopLimit ().IsStrictlyPositive () && m_currentMpdu->GetHeader ().IsData ())
    {
      needTxopFragmentation = (GetLow ()->CalculateOverallTxTime (m_currentMpdu, m_currentParams) > GetTxopLimit ());
    }
  return (needTxopFragmentation || m_stationManager->NeedFragmentation (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                                        &m_currentMpdu->GetHeader (),
                                                                        m_currentMpdu->GetPacket ()));
}

bool
QosTxop::IsTxopFragmentation (void) const
{
  if (GetTxopLimit ().IsZero ())
    {
      return false;
    }
  if (!m_stationManager->NeedFragmentation (m_currentMpdu->GetHeader ().GetAddr1 (), &m_currentMpdu->GetHeader (),
                                            m_currentMpdu->GetPacket ())
      || (GetTxopFragmentSize () < m_stationManager->GetFragmentSize (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                                      &m_currentMpdu->GetHeader (),
                                                                      m_currentMpdu->GetPacket (), 0)))
    {
      return true;
    }
  return false;
}

uint32_t
QosTxop::GetTxopFragmentSize (void) const
{
  Time txopDuration = GetTxopLimit ();
  if (txopDuration.IsZero ())
    {
      return 0;
    }
  uint32_t maxSize = m_currentMpdu->GetPacket ()->GetSize ();
  uint32_t minSize = 0;
  uint32_t size = 0;
  bool found = false;
  while (!found)
    {
      size = (minSize + ((maxSize - minSize) / 2));
      if (GetLow ()->CalculateOverallTxTime (m_currentMpdu, m_currentParams, size) > txopDuration)
        {
          maxSize = size;
        }
      else
        {
          minSize = size;
        }
      if (GetLow ()->CalculateOverallTxTime (m_currentMpdu, m_currentParams, size) <= txopDuration
          && GetLow ()->CalculateOverallTxTime (m_currentMpdu, m_currentParams, size + 1) > txopDuration)
        {
          found = true;
        }
    }
  NS_ASSERT (size != 0);
  return size;
}

uint32_t
QosTxop::GetNTxopFragment (void) const
{
  uint32_t fragmentSize = GetTxopFragmentSize ();
  uint32_t nFragments = (m_currentMpdu->GetPacket ()->GetSize () / fragmentSize);
  if ((m_currentMpdu->GetPacket ()->GetSize () % fragmentSize) > 0)
    {
      nFragments++;
    }
  NS_LOG_DEBUG ("GetNTxopFragment returning " << nFragments);
  return nFragments;
}

uint32_t
QosTxop::GetTxopFragmentOffset (uint32_t fragmentNumber) const
{
  if (fragmentNumber == 0)
    {
      return 0;
    }
  uint32_t offset = 0;
  uint32_t fragmentSize = GetTxopFragmentSize ();
  uint32_t nFragments = (m_currentMpdu->GetPacket ()->GetSize () / fragmentSize);
  if ((m_currentMpdu->GetPacket ()->GetSize () % fragmentSize) > 0)
    {
      nFragments++;
    }
  if (fragmentNumber < nFragments)
    {
      offset = (fragmentNumber * fragmentSize);
    }
  else
    {
      NS_ASSERT (false);
    }
  NS_LOG_DEBUG ("GetTxopFragmentOffset returning " << offset);
  return offset;
}

uint32_t
QosTxop::GetNextTxopFragmentSize (uint32_t fragmentNumber) const
{
  NS_LOG_FUNCTION (this << fragmentNumber);
  uint32_t fragmentSize = GetTxopFragmentSize ();
  uint32_t nFragments = GetNTxopFragment ();
  if (fragmentNumber >= nFragments)
    {
      NS_LOG_DEBUG ("GetNextTxopFragmentSize returning 0");
      return 0;
    }
  if (fragmentNumber == nFragments - 1)
    {
      fragmentSize = (m_currentMpdu->GetPacket ()->GetSize () - ((nFragments - 1) * fragmentSize));
    }
  NS_LOG_DEBUG ("GetNextTxopFragmentSize returning " << fragmentSize);
  return fragmentSize;
}

uint32_t
QosTxop::GetFragmentSize (void) const
{
  uint32_t size;
  if (IsTxopFragmentation ())
    {
      size = GetNextTxopFragmentSize (m_fragmentNumber);
    }
  else
    {
      size = m_stationManager->GetFragmentSize (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                &m_currentMpdu->GetHeader (),
                                                m_currentMpdu->GetPacket (), m_fragmentNumber);
    }
  return size;
}

uint32_t
QosTxop::GetNextFragmentSize (void) const
{
  uint32_t size;
  if (IsTxopFragmentation ())
    {
      size = GetNextTxopFragmentSize (m_fragmentNumber + 1);
    }
  else
    {
      size = m_stationManager->GetFragmentSize (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                &m_currentMpdu->GetHeader (),
                                                m_currentMpdu->GetPacket (), m_fragmentNumber + 1);
    }
  return size;
}

uint32_t
QosTxop::GetFragmentOffset (void) const
{
  uint32_t offset;
  if (IsTxopFragmentation ())
    {
      offset = GetTxopFragmentOffset (m_fragmentNumber);
    }
  else
    {
      offset = m_stationManager->GetFragmentOffset (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                    &m_currentMpdu->GetHeader (),
                                                    m_currentMpdu->GetPacket (), m_fragmentNumber);
    }
  return offset;
}

bool
QosTxop::IsLastFragment (void) const
{
  bool isLastFragment;
  if (IsTxopFragmentation ())
    {
      isLastFragment = (m_fragmentNumber == GetNTxopFragment () - 1);
    }
  else
    {
      isLastFragment = m_stationManager->IsLastFragment (m_currentMpdu->GetHeader ().GetAddr1 (),
                                                         &m_currentMpdu->GetHeader (),
                                                         m_currentMpdu->GetPacket (), m_fragmentNumber);
    }
  return isLastFragment;
}

void
QosTxop::SetAccessCategory (AcIndex ac)
{
  NS_LOG_FUNCTION (this << +ac);
  m_ac = ac;
}

Mac48Address
QosTxop::MapSrcAddressForAggregation (const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << &hdr);
  Mac48Address retval;
  if (GetTypeOfStation () == STA || GetTypeOfStation () == ADHOC_STA)
    {
      retval = hdr.GetAddr2 ();
    }
  else
    {
      retval = hdr.GetAddr3 ();
    }
  return retval;
}

Mac48Address
QosTxop::MapDestAddressForAggregation (const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << &hdr);
  Mac48Address retval;
  if (GetTypeOfStation () == AP || GetTypeOfStation () == ADHOC_STA)
    {
      retval = hdr.GetAddr1 ();
    }
  else
    {
      retval = hdr.GetAddr3 ();
    }
  return retval;
}

void
QosTxop::PushFront (Ptr<WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << *mpdu);
  m_stationManager->PrepareForQueue (mpdu->GetHeader ().GetAddr1 (), &mpdu->GetHeader (), mpdu->GetPacket ());
  m_queue->PushFront (mpdu);
  StartAccessIfNeeded ();
}

void
QosTxop::GotAddBaResponse (const MgtAddBaResponseHeader *respHdr, Mac48Address recipient)
{
  NS_LOG_FUNCTION (this << respHdr << recipient);
  uint8_t tid = respHdr->GetTid ();
  if (respHdr->GetStatusCode ().IsSuccess ())
    {
      NS_LOG_DEBUG ("block ack agreement established with " << recipient);
      // Even though a (destination, TID) pair is "blocked" (i.e., no more packets
      // are sent) when an Add BA Request is sent to the destination,
      // the current packet may still be non-null when the Add BA Response is received.
      // In fact, if the Add BA Request timer expires, the (destination, TID) pair is
      // "unblocked" and packets to the destination are sent again (under normal
      // ack policy). Thus, there may be a packet needing to be retransmitted
      // when the Add BA Response is received. If this is the case, let the Block
      // Ack manager handle its retransmission.
      if (m_currentMpdu != 0 && m_currentMpdu->GetHeader ().IsQosData ()
          && m_currentMpdu->GetHeader ().GetAddr1 () == recipient && m_currentMpdu->GetHeader ().GetQosTid () == tid)
        {
          m_baManager->GetRetransmitQueue ()->Enqueue (m_currentMpdu);
          m_currentMpdu = 0;
        }
      m_baManager->UpdateAgreement (respHdr, recipient);
    }
  else
    {
      NS_LOG_DEBUG ("discard ADDBA response" << recipient);
      m_baManager->NotifyAgreementRejected (recipient, tid);
    }
  RestartAccessIfNeeded ();
}

void
QosTxop::GotDelBaFrame (const MgtDelBaHeader *delBaHdr, Mac48Address recipient)
{
  NS_LOG_FUNCTION (this << delBaHdr << recipient);
  NS_LOG_DEBUG ("received DELBA frame from=" << recipient);
  m_baManager->DestroyAgreement (recipient, delBaHdr->GetTid ());
}

void
QosTxop::GotBlockAck (const CtrlBAckResponseHeader *blockAck, Mac48Address recipient, double rxSnr, WifiMode txMode, double dataSnr)
{
  NS_LOG_FUNCTION (this << blockAck << recipient << rxSnr << txMode.GetUniqueName () << dataSnr);
  NS_LOG_DEBUG ("got block ack from=" << recipient);

  if (blockAck->IsMultiSta ())
    {
      uint16_t aid = m_low->GetStaId (m_low->GetAddress ());
      for (auto& index : blockAck->FindPerAidTidInfoWithAid (aid))
        {
          uint8_t tid = blockAck->GetTidInfo (index);

          if (blockAck->GetAckType (index))
            {
              // Acknowledgment context
              NS_ASSERT (m_currentMpdu != 0 && m_currentMpdu->GetHeader ().IsQosData ()
                         && m_currentMpdu->GetHeader ().GetQosTid () == tid);
              GetBaManager (tid)->NotifyGotAck (m_currentMpdu);
            }
          else
            {
              // Block Acknowledgment context
              GetBaManager (tid)->NotifyGotBlockAck (blockAck, recipient, rxSnr, txMode, dataSnr, index);
            }
        }
    }
  else
    {
      uint8_t tid = blockAck->GetTidInfo ();
      GetBaManager (tid)->NotifyGotBlockAck (blockAck, recipient, rxSnr, txMode, dataSnr);
    }
  m_currentMpdu = 0;
  ResetCw ();
}

bool QosTxop::GetAmpduExist (Mac48Address dest) const
{
  NS_LOG_FUNCTION (this << dest);
  auto it = m_aMpduEnabled.find (dest);
  if (it != m_aMpduEnabled.end ())
    {
      return it->second;
    }
  return false;
}

void QosTxop::SetAmpduExist (Mac48Address dest, bool enableAmpdu)
{
  NS_LOG_FUNCTION (this << dest << enableAmpdu);
  m_aMpduEnabled[dest] = enableAmpdu;
}

void
QosTxop::CompleteMpduTx (Ptr<WifiMacQueueItem> mpdu)
{
  NS_ASSERT (mpdu->GetHeader ().IsQosData ());
  // If there is an established BA agreement, store the packet in the queue of outstanding packets
  if (GetBaAgreementEstablished (mpdu->GetHeader ().GetAddr1 (), mpdu->GetHeader ().GetQosTid ()))
    {
      m_baManager->StorePacket (mpdu);
    }
}

bool
QosTxop::SetupBlockAckIfNeeded (void)
{
  NS_LOG_FUNCTION (this);
  uint8_t tid = m_currentMpdu->GetHeader ().GetQosTid ();
  Mac48Address recipient = m_currentMpdu->GetHeader ().GetAddr1 ();
  uint32_t packets = m_queue->GetNPacketsByTidAndAddress (tid, recipient);
  if ((GetBlockAckThreshold () > 0 && packets >= GetBlockAckThreshold ())
      || (GetLow ()->GetMpduAggregator () != 0 && GetLow ()->GetMpduAggregator ()->GetMaxAmpduSize (recipient, tid, WIFI_MOD_CLASS_HT) > 0 && packets > 1)
      || m_stationManager->GetVhtSupported ()
      || m_stationManager->GetHeSupported ())
    {
      /* Block ack setup */
      uint16_t startingSequence = m_txMiddle->GetNextSeqNumberByTidAndAddress (tid, recipient);
      SendAddBaRequest (recipient, tid, startingSequence, m_blockAckInactivityTimeout, true);
      return true;
    }
  return false;
}

void
QosTxop::CompleteConfig (void)
{
  NS_LOG_FUNCTION (this);
  m_baManager->SetTxMiddle (m_txMiddle);
  m_low->RegisterEdcaForAc (m_ac, this);
  m_baManager->SetBlockAckInactivityCallback (MakeCallback (&QosTxop::SendDelbaFrame, this));
}

void
QosTxop::SetBlockAckThreshold (uint8_t threshold)
{
  NS_LOG_FUNCTION (this << +threshold);
  m_blockAckThreshold = threshold;
  m_baManager->SetBlockAckThreshold (threshold);
}

void
QosTxop::SetBlockAckInactivityTimeout (uint16_t timeout)
{
  NS_LOG_FUNCTION (this << timeout);
  m_blockAckInactivityTimeout = timeout;
}

uint8_t
QosTxop::GetBlockAckThreshold (void) const
{
  NS_LOG_FUNCTION (this);
  return m_blockAckThreshold;
}

void
QosTxop::SendAddBaRequest (Mac48Address dest, uint8_t tid, uint16_t startSeq,
                           uint16_t timeout, bool immediateBAck)
{
  NS_LOG_FUNCTION (this << dest << +tid << startSeq << timeout << immediateBAck);
  NS_LOG_DEBUG ("sent ADDBA request to " << dest);
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_MGT_ACTION);
  hdr.SetAddr1 (dest);
  hdr.SetAddr2 (m_low->GetAddress ());
  hdr.SetAddr3 (m_low->GetAddress ());
  hdr.SetDsNotTo ();
  hdr.SetDsNotFrom ();

  WifiActionHeader actionHdr;
  WifiActionHeader::ActionValue action;
  action.blockAck = WifiActionHeader::BLOCK_ACK_ADDBA_REQUEST;
  actionHdr.SetAction (WifiActionHeader::BLOCK_ACK, action);

  Ptr<Packet> packet = Create<Packet> ();
  /*Setting ADDBARequest header*/
  MgtAddBaRequestHeader reqHdr;
  reqHdr.SetAmsduSupport (true);
  if (immediateBAck)
    {
      reqHdr.SetImmediateBlockAck ();
    }
  else
    {
      reqHdr.SetDelayedBlockAck ();
    }
  reqHdr.SetTid (tid);
  /* For now we don't use buffer size field in the ADDBA request frame. The recipient
   * will choose how many packets it can receive under block ack.
   */
  reqHdr.SetBufferSize (0);
  reqHdr.SetTimeout (timeout);
  reqHdr.SetStartingSequence (startSeq);

  m_baManager->CreateAgreement (&reqHdr, dest);

  packet->AddHeader (reqHdr);
  packet->AddHeader (actionHdr);

  m_currentMpdu = Create<WifiMacQueueItem> (packet, hdr);

  uint16_t sequence = m_txMiddle->GetNextSequenceNumberFor (&m_currentMpdu->GetHeader ());
  m_currentMpdu->GetHeader ().SetSequenceNumber (sequence);
  m_stationManager->UpdateFragmentationThreshold ();
  m_currentMpdu->GetHeader ().SetFragmentNumber (0);
  m_currentMpdu->GetHeader ().SetNoMoreFragments ();
  m_currentMpdu->GetHeader ().SetNoRetry ();

  m_currentParams.EnableAck ();
  m_currentParams.DisableRts ();
  m_currentParams.DisableNextData ();

  m_low->StartTransmission (m_currentMpdu, m_currentParams, this);
}

void
QosTxop::SendDelbaFrame (Mac48Address addr, uint8_t tid, bool byOriginator)
{
  NS_LOG_FUNCTION (this << addr << +tid << byOriginator);
  WifiMacHeader hdr;
  hdr.SetType (WIFI_MAC_MGT_ACTION);
  hdr.SetAddr1 (addr);
  hdr.SetAddr2 (m_low->GetAddress ());
  hdr.SetAddr3 (m_low->GetAddress ());
  hdr.SetDsNotTo ();
  hdr.SetDsNotFrom ();

  MgtDelBaHeader delbaHdr;
  delbaHdr.SetTid (tid);
  if (byOriginator)
    {
      delbaHdr.SetByOriginator ();
      m_baManager->DestroyAgreement (addr, tid);
    }
  else
    {
      delbaHdr.SetByRecipient ();
      m_low->DestroyBlockAckAgreement (addr, tid);
    }

  WifiActionHeader actionHdr;
  WifiActionHeader::ActionValue action;
  action.blockAck = WifiActionHeader::BLOCK_ACK_DELBA;
  actionHdr.SetAction (WifiActionHeader::BLOCK_ACK, action);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (delbaHdr);
  packet->AddHeader (actionHdr);

  PushFront (Create<WifiMacQueueItem> (packet, hdr));
}

void
QosTxop::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  ResetCw ();
  m_cwTrace = GetCw ();
  m_backoff = m_rng->GetInteger (0, GetCw ());
  m_backoffTrace (m_backoff);
  StartBackoffNow (m_backoff);
}

void
QosTxop::BaTxOk (const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << hdr);
  if (!m_txOkCallback.IsNull ())
    {
      m_txOkCallback (hdr);
    }
}

void
QosTxop::BaTxFailed (const WifiMacHeader &hdr)
{
  NS_LOG_FUNCTION (this << hdr);
  if (!m_txFailedCallback.IsNull ())
    {
      m_txFailedCallback (hdr);
    }
}

void
QosTxop::AddBaResponseTimeout (Mac48Address recipient, uint8_t tid)
{
  NS_LOG_FUNCTION (this << recipient << +tid);
  // If agreement is still pending, ADDBA response is not received
  if (m_baManager->ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::PENDING))
    {
      m_baManager->NotifyAgreementNoReply (recipient, tid);
      Simulator::Schedule (m_failedAddBaTimeout, &QosTxop::ResetBa, this, recipient, tid);
      m_backoff = m_rng->GetInteger (0, GetCw ());
      m_backoffTrace (m_backoff);
      StartBackoffNow (m_backoff);
      RestartAccessIfNeeded ();
    }
}

void
QosTxop::ResetBa (Mac48Address recipient, uint8_t tid)
{
  NS_LOG_FUNCTION (this << recipient << +tid);
  // This function is scheduled when waiting for an ADDBA response. However,
  // before this function is called, a DELBA request may arrive, which causes
  // the agreement to be deleted. Hence, check if an agreement exists before
  // notifying that the agreement has to be reset.
  if (m_baManager->ExistsAgreement (recipient, tid)
      && !m_baManager->ExistsAgreementInState (recipient, tid, OriginatorBlockAckAgreement::ESTABLISHED))
    {
      m_baManager->NotifyAgreementReset (recipient, tid);
    }
}

void
QosTxop::SetAddBaResponseTimeout (Time addBaResponseTimeout)
{
  NS_LOG_FUNCTION (this << addBaResponseTimeout);
  m_addBaResponseTimeout = addBaResponseTimeout;
}

Time
QosTxop::GetAddBaResponseTimeout (void) const
{
  return m_addBaResponseTimeout;
}

void
QosTxop::SetFailedAddBaTimeout (Time failedAddBaTimeout)
{
  NS_LOG_FUNCTION (this << failedAddBaTimeout);
  m_failedAddBaTimeout = failedAddBaTimeout;
}

Time
QosTxop::GetFailedAddBaTimeout (void) const
{
  return m_failedAddBaTimeout;
}

bool
QosTxop::IsQosTxop (void) const
{
  return true;
}

} //namespace ns3
