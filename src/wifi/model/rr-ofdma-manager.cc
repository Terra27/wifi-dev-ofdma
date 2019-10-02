/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019 Universita' degli Studi di Napoli Federico II
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
 * Author: Stefano Avallone <stavallo@unina.it>
 */

#include "ns3/log.h"
#include "rr-ofdma-manager.h"
#include "wifi-ack-policy-selector.h"
#include "wifi-phy.h"
#include <utility>
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RrOfdmaManager");

NS_OBJECT_ENSURE_REGISTERED (RrOfdmaManager);

TypeId
RrOfdmaManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RrOfdmaManager")
    .SetParent<OfdmaManager> ()
    .SetGroupName ("Wifi")
    .AddConstructor<RrOfdmaManager> ()
    .AddAttribute ("NStations",
                   "The maximum number of stations that can be granted an RU in the MU DL OFDMA transmission",
                   UintegerValue (4),
                   MakeUintegerAccessor (&RrOfdmaManager::m_nStations),
                   MakeUintegerChecker<uint8_t> (1, 74))
    .AddAttribute ("ForceDlOfdma",
                   "If enabled, return DL_OFDMA even if no DL MU PPDU could be built.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&RrOfdmaManager::m_forceDlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableUlOfdma",
                   "If enabled, return UL_OFDMA if DL_OFDMA was returned the previous time.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RrOfdmaManager::m_enableUlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("UlPsduSize",
                   "The size in bytes of the solicited PSDU (to be sent in an HE TB PPDU)",
                   UintegerValue (500),
                   MakeUintegerAccessor (&RrOfdmaManager::m_ulPsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ChannelBw",
                   "For TESTING only",
                   UintegerValue (20),
                   MakeUintegerAccessor (&RrOfdmaManager::m_bw),
                   MakeUintegerChecker<uint16_t> (5, 160))
  ;
  return tid;
}

RrOfdmaManager::RrOfdmaManager ()
  : m_startStation (0)
{
  NS_LOG_FUNCTION (this);
}

RrOfdmaManager::~RrOfdmaManager ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
RrOfdmaManager::InitTxVectorAndParams (std::map<Mac48Address, DlPerStaInfo> staList,
                                         HeRu::RuType ruType, DlMuAckSequenceType dlMuAckSequence)
{
  NS_LOG_FUNCTION (this);
  m_txVector = WifiTxVector ();
  m_txVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
  m_txVector.SetChannelWidth (m_low->GetPhy ()->GetChannelWidth ());
  m_txVector.SetGuardInterval (m_low->GetPhy ()->GetGuardInterval ().GetNanoSeconds ());
  m_txVector.SetTxPowerLevel (GetWifiRemoteStationManager ()->GetDefaultTxPowerLevel ());
  m_txParams = MacLowTransmissionParameters ();
  m_txParams.SetDlMuAckSequenceType (dlMuAckSequence);

  Ptr<WifiMacQueueItem> mpdu = Copy (m_mpdu);

  for (auto& sta : staList)
    {
      mpdu->GetHeader ().SetAddr1 (sta.first);
      // Get the TX vector used to transmit single user frames to the receiver
      // station (the RU index will be assigned by ComputeDlOfdmaInfo)
      WifiTxVector suTxVector = m_low->GetDataTxVector (mpdu);
      NS_LOG_DEBUG ("Adding STA with AID=" << sta.second.aid << " and TX mode="
                    << suTxVector.GetMode () << " to the TX vector");
      m_txVector.SetHeMuUserInfo (sta.second.aid, {{false, ruType, 1}, suTxVector.GetMode (), suTxVector.GetNss ()});

      // Add the receiver station to the appropriate list of the TX params
      Ptr<QosTxop> txop = m_qosTxop[QosUtilsMapTidToAc (sta.second.tid)];
      BlockAckReqType barType = txop->GetBaAgreementEstablished (sta.first, sta.second.tid)
                                ? txop->GetBlockAckReqType (sta.first, sta.second.tid)
                                : BlockAckReqType::COMPRESSED;
      BlockAckType baType = txop->GetBaAgreementEstablished (sta.first, sta.second.tid)
                            ? txop->GetBlockAckType (sta.first, sta.second.tid)
                            : BlockAckType::COMPRESSED;

      if (dlMuAckSequence == DlMuAckSequenceType::DL_SU_FORMAT)
        {
          // Enable BAR/BA exchange for all the receiver stations
          m_txParams.EnableBlockAckRequest (sta.first, barType, baType);
        }
      else if (dlMuAckSequence == DlMuAckSequenceType::DL_MU_BAR)
        {
          // Send a MU-BAR to all the stations
          m_txParams.EnableBlockAckRequest (sta.first, barType, baType);
        }
      else if (dlMuAckSequence == DlMuAckSequenceType::DL_AGGREGATE_TF)
        {
          // Expect to receive a Block Ack from all the stations
          m_txParams.EnableBlockAck (sta.first, baType);
        }
    }
}

OfdmaTxFormat
RrOfdmaManager::SelectTxFormat (Ptr<const WifiMacQueueItem> mpdu)
{
  // --- for TESTING only ---
//   for (uint8_t i = 1; i <= m_nStations; i++)
//     {
//       DlPerStaInfo info {i, 0};
//       m_staInfo.push_back (std::make_pair (Mac48Address::Allocate (), info));
//     }
//   return OfdmaTxFormat::DL_OFDMA;
  // --- --- ---
  NS_LOG_FUNCTION (this << *mpdu);
  NS_ASSERT (mpdu->GetHeader ().IsQosData ());

  if (m_enableUlOfdma && GetTxFormat () == DL_OFDMA)
    {
      // check if an UL OFDMA transmission is possible after a DL OFDMA transmission
      NS_ABORT_MSG_IF (m_ulPsduSize == 0, "The UlPsduSize attribute must be set to a non-null value");

      Ptr<QosTxop> txop = m_qosTxop[QosUtilsMapTidToAc (mpdu->GetHeader ().GetQosTid ())];
      m_ulMuAckSequence = txop->GetAckPolicySelector ()->GetAckSequenceForUlMu ();
      MacLowTransmissionParameters params;
      params.SetUlMuAckSequenceType (m_ulMuAckSequence);
      BlockAckType baType;

      if (m_ulMuAckSequence == UL_MULTI_STA_BLOCK_ACK)
        {
          baType = BlockAckType::MULTI_STA;
          for (auto& userInfo : m_txVector.GetHeMuUserInfoMap ())
            {
              auto addressIt = m_apMac->GetStaList ().find (userInfo.first);
              if (addressIt != m_apMac->GetStaList ().end ())
                {
                  baType.m_bitmapLen.push_back (32);
                  params.EnableBlockAck (addressIt->second, baType);
                }
              else
                {
                  NS_LOG_WARN ("Maybe station with AID=" << userInfo.first << " left the BSS since the last MU DL transmission?");
                }
            }
        }
      else
        {
          NS_FATAL_ERROR ("Sending Block Acks in an MU DL PPDU is not supported yet");
        }

      CtrlTriggerHeader trigger (TriggerFrameType::BASIC_TRIGGER, m_txVector);

      // compute the maximum amount of time that can be granted to stations.
      // This value is limited by the max PPDU duration
      Time maxDuration = GetPpduMaxTime (m_txVector.GetPreambleType ());

      // compute the time required by stations based on the buffer status reports, if any
      uint32_t maxBufferSize = 0;

      for (auto& userInfo : m_txVector.GetHeMuUserInfoMap ())
        {
          auto addressIt = m_apMac->GetStaList ().find (userInfo.first);
          if (addressIt != m_apMac->GetStaList ().end ())
            {
              uint8_t queueSize = m_apMac->GetMaxBufferStatus (addressIt->second);
              if (queueSize == 255)
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is unknown");
                  maxBufferSize = std::max (maxBufferSize, m_ulPsduSize);
                }
              else if (queueSize == 254)
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is not limited");
                  maxBufferSize = 0xffffffff;
                  break;
                }
              else
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is " << +queueSize);
                  maxBufferSize = std::max (maxBufferSize, static_cast<uint32_t> (queueSize * 256));
                }
            }
          else
            {
              NS_LOG_WARN ("Maybe station with AID=" << userInfo.first << " left the BSS since the last MU DL transmission?");
            }
        }

      // if the maximum buffer size is 0, skip UL OFDMA and proceed with trying DL OFDMA
      if (maxBufferSize > 0)
        {
          // if we are within a TXOP, we have to consider the response time and the
          // remaining TXOP duration
          if (txop->GetTxopLimit ().IsStrictlyPositive ())
            {
              // we need to define the HE TB PPDU duration in order to compute the response to
              // the Trigger Frame. Let's use 1 ms for this purpose. We'll subtract it later.
              uint16_t length = WifiPhy::ConvertHeTbPpduDurationToLSigLength (MilliSeconds (1),
                                                                              m_low->GetPhy ()->GetFrequency ());
              trigger.SetUlLength (length);

              Ptr<Packet> packet = Create<Packet> ();
              packet->AddHeader (trigger);
              WifiMacHeader hdr;
              hdr.SetType (WIFI_MAC_CTL_TRIGGER);
              hdr.SetAddr1 (Mac48Address::GetBroadcast ());
              Ptr<WifiMacQueueItem> item = Create<WifiMacQueueItem> (packet, hdr);

              Time response = m_low->GetResponseDuration (params, m_txVector, item);

              // Add the time to transmit the Trigger Frame itself
              WifiTxVector txVector = GetWifiRemoteStationManager ()->GetRtsTxVector (hdr.GetAddr1 (), &hdr, packet);

              response += m_low->GetPhy ()->CalculateTxDuration (item->GetSize (), txVector,
                                                                 m_low->GetPhy ()->GetFrequency ());

              // Subtract the duration of the HE TB PPDU
              response -= WifiPhy::ConvertLSigLengthToHeTbPpduDuration (length, m_txVector,
                                                                        m_low->GetPhy ()->GetFrequency ());

              if (response > txop->GetTxopRemaining ())
                {
                  // an UL OFDMA transmission is not possible. Reset m_staInfo and return DL_OFDMA.
                  // In this way, no transmission will occur now and the next time we will try again
                  // performing an UL OFDMA transmission.
                  NS_LOG_DEBUG ("Remaining TXOP duration is not enough for UL MU exchange");
                  m_staInfo.clear ();
                  return DL_OFDMA;
                }

              maxDuration = Min (maxDuration, txop->GetTxopRemaining () - response);
            }

          Time bufferTxTime = m_low->GetPhy ()->CalculateTxDuration (maxBufferSize, m_txVector,
                                                                     m_low->GetPhy ()->GetFrequency (),
                                                                     trigger.begin ()->GetAid12 ());
          if (bufferTxTime < maxDuration)
            {
              // the maximum buffer size can be transmitted within the allowed time
              maxDuration = bufferTxTime;
            }
          else
            {
              // maxDuration may be a too short time. If it does not allow to transmit
              // at least m_ulPsduSize bytes, give up the UL MU transmission for now
              Time minDuration = m_low->GetPhy ()->CalculateTxDuration (m_ulPsduSize, m_txVector,
                                                                        m_low->GetPhy ()->GetFrequency (),
                                                                        trigger.begin ()->GetAid12 ());
              if (maxDuration < minDuration)
                {
                  // maxDuration is a too short time. Reset m_staInfo and return DL_OFDMA.
                  // In this way, no transmission will occur now and the next time we will try again
                  // performing an UL OFDMA transmission.
                  NS_LOG_DEBUG ("Available time " << maxDuration << " is too short");
                  m_staInfo.clear ();
                  return DL_OFDMA;
                }
            }

          // maxDuration is the time to grant to the stations. Store it in the TX vector
          NS_LOG_DEBUG ("HE TB PPDU duration: " << maxDuration.ToDouble (Time::MS));
          uint16_t length = WifiPhy::ConvertHeTbPpduDurationToLSigLength (maxDuration,
                                                                          m_low->GetPhy ()->GetFrequency ());
          m_txVector.SetLength (length);
          m_txParams = params;
          return UL_OFDMA;
        }
    }

  // get the list of associated stations ((AID, MAC address) pairs)
  const std::map<uint16_t, Mac48Address>& staList = m_apMac->GetStaList ();
  auto startIt = staList.find (m_startStation);

  // This may be the first invocation or the starting station left
  if (startIt == staList.end ())
    {
      startIt = staList.begin ();
      m_startStation = startIt->first;
    }

  uint8_t currTid = mpdu->GetHeader ().GetQosTid ();
  AcIndex primaryAc = QosUtilsMapTidToAc (currTid);
  m_staInfo.clear ();

  // If the primary AC holds a TXOP, we can select a station as a receiver of
  // the MU PPDU only if the AP has frames to send to such station that fit into
  // the remaining TXOP time. To this end, we need to determine the type of ack
  // sequence and the time it takes. To compute the latter, we can call the
  // MacLow::GetResponseDuration () method, which requires TX vector and TX params.
  // Our best guess at this stage is that the AP has frames to send to all the
  // associated stations and hence we initialize the TX vector and the TX params
  // by considering the starting station and those that immediately follow it in
  // the list of associated stations.
  std::size_t count = m_nStations;
  HeRu::RuType ruType = GetNumberAndTypeOfRus (m_low->GetPhy ()->GetChannelWidth (), count);
  NS_ASSERT (count >= 1);

  std::map<Mac48Address, DlPerStaInfo> guess;
  auto staIt = startIt;
  do
    {
      guess[staIt->second] = {staIt->first, currTid};
      if (++staIt == staList.end ())
        {
          staIt = staList.begin ();
        }
    } while (guess.size () < count && staIt != startIt);

  Ptr<WifiAckPolicySelector> ackSelector = m_qosTxop[primaryAc]->GetAckPolicySelector ();
  NS_ASSERT (ackSelector != 0);
  m_dlMuAckSequence = ackSelector->GetAckSequenceForDlMu ();
  InitTxVectorAndParams (guess, ruType, m_dlMuAckSequence);

  // if the AC owns a TXOP, compute the time available for the transmission of data frames
  Time txopLimit = Seconds (0);
  if (m_qosTxop[primaryAc]->GetTxopLimit ().IsStrictlyPositive ())
    {
      // TODO Account for MU-RTS/CTS when implemented
      CtrlTriggerHeader trigger;

      if (m_dlMuAckSequence == DlMuAckSequenceType::DL_MU_BAR
          || m_dlMuAckSequence == DlMuAckSequenceType::DL_AGGREGATE_TF)
        {
          // Need to prepare the MU-BAR to correctly get the response time
          trigger = GetTriggerFrameHeader (m_txVector, 5);
          trigger.SetUlLength (m_low->CalculateUlLengthForBlockAcks (trigger, m_txParams));
        }
      txopLimit = m_qosTxop[primaryAc]->GetTxopRemaining () - GetResponseDuration (m_txParams, m_txVector, trigger);

      if (txopLimit.IsNegative ())
        {
          if (m_forceDlOfdma)
            {
              NS_LOG_DEBUG ("Not enough TXOP remaining time: return DL_OFDMA with empty set of receiver stations");
              return OfdmaTxFormat::DL_OFDMA;
            }
          NS_LOG_DEBUG ("Not enough TXOP remaining time: return NON_OFDMA");
          return OfdmaTxFormat::NON_OFDMA;
        }
    }

  // iterate over the associated stations until an enough number of stations is identified
  do
    {
      NS_LOG_DEBUG ("Next candidate STA (MAC=" << startIt->second << ", AID=" << startIt->first << ")");
      // check if the AP has at least one frame to be sent to the current station
      for (uint8_t tid : std::initializer_list<uint8_t> {currTid, 1, 2, 0, 3, 4, 5, 6, 7})
        {
          AcIndex ac = QosUtilsMapTidToAc (tid);
          // check that a BA agreement is established with the receiver for the
          // considered TID, since ack sequences for DL MU PPDUs require block ack
          if (ac >= primaryAc && m_qosTxop[ac]->GetBaAgreementEstablished (startIt->second, tid))
            {
              Ptr<const WifiMacQueueItem> mpdu;
              mpdu = m_qosTxop[ac]->PeekNextFrame (tid, startIt->second);

              // we only check if the first frame of the current TID meets the size
              // and duration constraints. We do not explore the queues further.
              if (mpdu != 0)
                {
                  // Use a temporary TX vector including only the STA-ID of the
                  // candidate station to check if the MPDU meets the size and time limits.
                  // An RU of the computed size is tentatively assigned to the candidate
                  // station, so that the TX duration can be correctly computed.
                  WifiTxVector suTxVector = m_low->GetDataTxVector (mpdu),
                               muTxVector;

                  muTxVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
                  muTxVector.SetChannelWidth (m_low->GetPhy ()->GetChannelWidth ());
                  muTxVector.SetGuardInterval (m_low->GetPhy ()->GetGuardInterval ().GetNanoSeconds ());
                  muTxVector.SetHeMuUserInfo (startIt->first,
                                              {{false, ruType, 1}, suTxVector.GetMode (), suTxVector.GetNss ()});

                  if (m_low->IsWithinSizeAndTimeLimits (mpdu, muTxVector, 0, txopLimit))
                    {
                      // the frame meets the constraints, add the station to the list
                      NS_LOG_DEBUG ("Adding candidate STA (MAC=" << startIt->second << ", AID="
                                    << startIt->first << ") TID=" << +tid);
                      DlPerStaInfo info {startIt->first, tid};
                      m_staInfo.push_back (std::make_pair (startIt->second, info));
                      break;    // terminate the for loop
                    }
                }
              else
                {
                  NS_LOG_DEBUG ("No frames to send to " << startIt->second << " with TID=" << +tid);
                }
            }
        }

      // move to the next station in the map
      startIt++;
      if (startIt == staList.end ())
        {
          startIt = staList.begin ();
        }
    } while (m_staInfo.size () < m_nStations && startIt->first != m_startStation);

  if (m_staInfo.empty ())
    {
      if (m_forceDlOfdma)
        {
          NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return DL_OFDMA with empty set of receiver stations");
          return OfdmaTxFormat::DL_OFDMA;
        }
      NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return NON_OFDMA");
      return OfdmaTxFormat::NON_OFDMA;
    }

  m_startStation = startIt->first;
  return OfdmaTxFormat::DL_OFDMA;
}

HeRu::RuType
RrOfdmaManager::GetNumberAndTypeOfRus (uint16_t bandwidth, std::size_t& nStations) const
{
  NS_LOG_FUNCTION (this);

  HeRu::RuType ruType;
  uint8_t nRusAssigned = 0;

  // iterate over all the available RU types
  for (auto& ru : HeRu::m_heRuSubcarrierGroups)
    {
      if (ru.first.first == bandwidth && ru.second.size () <= nStations)
        {
          ruType = ru.first.second;
          nRusAssigned = ru.second.size ();
          break;
        }
      else if (bandwidth == 160 && ru.first.first == 80 && (2 * ru.second.size () <= nStations))
        {
          ruType = ru.first.second;
          nRusAssigned = 2 * ru.second.size ();
          break;
        }
    }
  if (nRusAssigned == 0)
    {
      NS_ASSERT (bandwidth == 160 && nStations == 1);
      nRusAssigned = 1;
      ruType = HeRu::RU_2x996_TONE;
    }

  nStations = nRusAssigned;
  return ruType;
}

OfdmaManager::DlOfdmaInfo
RrOfdmaManager::ComputeDlOfdmaInfo (void)
{
  NS_LOG_FUNCTION (this);

  if (m_staInfo.empty ())
    {
      return DlOfdmaInfo ();
    }

  uint16_t bw = m_low->GetPhy ()->GetChannelWidth ();
//   uint16_t bw = m_bw;   // for TESTING only

  // compute how many stations can be granted an RU and the RU size
  std::size_t nRusAssigned = m_staInfo.size ();
  HeRu::RuType ruType = GetNumberAndTypeOfRus (bw, nRusAssigned);

  NS_LOG_DEBUG (nRusAssigned << " stations are being assigned a " << ruType << " RU");

  DlOfdmaInfo dlOfdmaInfo;
  auto staInfoIt = m_staInfo.begin (); // iterator over the list of candidate receivers

  for (std::size_t i = 0; i < nRusAssigned; i++)
    {
      NS_ASSERT (staInfoIt != m_staInfo.end ());
      dlOfdmaInfo.staInfo.insert (*staInfoIt);
      staInfoIt++;
    }

  // if not all the stations are assigned an RU, the first station to serve next
  // time is the first one that was not served this time
  if (nRusAssigned < m_staInfo.size ())
    {
      NS_ASSERT (staInfoIt != m_staInfo.end ());
      m_startStation = staInfoIt->second.aid;
    }
  NS_LOG_DEBUG ("Next station to serve has AID=" << m_startStation);

  // set TX vector and TX params
  InitTxVectorAndParams (dlOfdmaInfo.staInfo, ruType, m_dlMuAckSequence);
  dlOfdmaInfo.params = m_txParams;

  // assign RUs to stations

  if (ruType == HeRu::RU_2x996_TONE)
    {
      HeRu::RuSpec ru = {true, ruType, 1};
      NS_LOG_DEBUG ("STA " << m_staInfo.front ().first << " assigned " << ru);
      m_txVector.SetRu (ru, m_staInfo.front ().second.aid);
    }
  else
    {
      std::vector<bool> primary80MHzSet {true};

      if (bw == 160)
        {
          primary80MHzSet.push_back (false);
          bw = 80;
        }

      auto mapIt = dlOfdmaInfo.staInfo.begin ();

      for (auto primary80MHz : primary80MHzSet)
        {
          for (std::size_t ruIndex = 1; ruIndex <= HeRu::m_heRuSubcarrierGroups.at ({bw, ruType}).size (); ruIndex++)
            {
              NS_ASSERT (mapIt != dlOfdmaInfo.staInfo.end ());
              HeRu::RuSpec ru = {primary80MHz, ruType, ruIndex};
              NS_LOG_DEBUG ("STA " << mapIt->first << " assigned " << ru);
              m_txVector.SetRu (ru, mapIt->second.aid);
              mapIt++;
            }
        }
    }
  dlOfdmaInfo.txVector = m_txVector;

  if (m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_MU_BAR
      || m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF)
    {
      Ptr<WifiRemoteStationManager> stationManager = GetWifiRemoteStationManager ();
      // The Trigger Frame to be returned is built from the TX vector used for the DL MU PPDU
      // (i.e., responses will use the same set of RUs) and modified to ensure that responses
      // are sent at a rate not higher than MCS 5.
      dlOfdmaInfo.trigger = GetTriggerFrameHeader (dlOfdmaInfo.txVector, 5);
      dlOfdmaInfo.trigger.SetUlLength (m_low->CalculateUlLengthForBlockAcks (dlOfdmaInfo.trigger, m_txParams));
      SetTargetRssi (dlOfdmaInfo.trigger);
    }

  return dlOfdmaInfo;
}

CtrlTriggerHeader
RrOfdmaManager::GetTriggerFrameHeader (WifiTxVector dlMuTxVector, uint8_t maxMcs)
{
  auto userInfoMap = dlMuTxVector.GetHeMuUserInfoMap ();

  for (auto& userInfo : userInfoMap)
    {
      uint8_t mcs = std::min (userInfo.second.mcs.GetMcsValue (), maxMcs);
      dlMuTxVector.SetHeMuUserInfo (userInfo.first, {userInfo.second.ru,
                                                     WifiPhy::GetHeMcs (mcs),
                                                     userInfo.second.nss});
    }

  return CtrlTriggerHeader (TriggerFrameType::MU_BAR_TRIGGER, dlMuTxVector);
}

OfdmaManager::UlOfdmaInfo
RrOfdmaManager::ComputeUlOfdmaInfo (void)
{
  CtrlTriggerHeader trigger (TriggerFrameType::BASIC_TRIGGER, m_txVector);
  trigger.SetUlLength (m_txVector.GetLength ());
  SetTargetRssi (trigger);

  UlOfdmaInfo ulOfdmaInfo;
  ulOfdmaInfo.params = m_txParams;
  ulOfdmaInfo.trigger = trigger;

  return ulOfdmaInfo;
}

} //namespace ns3
