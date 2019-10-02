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
#include "ns3/abort.h"
#include "ns3/pointer.h"
#include "ofdma-manager.h"
#include "qos-txop.h"
#include "he-configuration.h"
#include "wifi-phy.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("OfdmaManager");

NS_OBJECT_ENSURE_REGISTERED (OfdmaManager);

TypeId
OfdmaManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::OfdmaManager")
    .SetParent<Object> ()
    .SetGroupName ("Wifi")
  ;
  return tid;
}

OfdmaManager::~OfdmaManager ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
OfdmaManager::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_apMac = 0;
  m_mpdu = 0;
  m_qosTxop.clear ();
  m_low = 0;
  m_dlInfo.staInfo.clear ();
  Object::DoDispose ();
}

void
OfdmaManager::NotifyNewAggregate ()
{
  NS_LOG_FUNCTION (this);
  if (m_apMac == 0)
    {
      Ptr<ApWifiMac> apMac = this->GetObject<ApWifiMac> ();
      //verify that it's a valid AP mac and that
      //the AP mac was not set before
      if (apMac != 0)
        {
          this->SetWifiMac (apMac);
        }
    }
  Object::NotifyNewAggregate ();
}

void
OfdmaManager::SetWifiMac (Ptr<ApWifiMac> mac)
{
  NS_LOG_FUNCTION (this << mac);
  m_apMac = mac;

  NS_ABORT_MSG_IF (m_apMac == 0 || m_apMac->GetHeConfiguration () == 0,
                   "OfdmaManager can only be installed on HE APs");

  PointerValue ptr;
  m_apMac->GetAttribute ("BE_Txop", ptr);
  m_qosTxop[AC_BE] = ptr.Get<QosTxop> ();

  m_apMac->GetAttribute ("BK_Txop", ptr);
  m_qosTxop[AC_BK] = ptr.Get<QosTxop> ();

  m_apMac->GetAttribute ("VI_Txop", ptr);
  m_qosTxop[AC_VI] = ptr.Get<QosTxop> ();

  m_apMac->GetAttribute ("VO_Txop", ptr);
  m_qosTxop[AC_VO] = ptr.Get<QosTxop> ();

  m_low = ptr.Get<QosTxop> ()->GetLow ();
  m_low->SetOfdmaManager (this);
}

void
OfdmaManager::NotifyAccessGranted (Ptr<const WifiMacQueueItem> mpdu)
{
  NS_LOG_FUNCTION (this << mpdu);

  m_mpdu = mpdu;
  m_TxFormat = SelectTxFormat (m_mpdu);

  if (m_TxFormat == DL_OFDMA)
    {
      m_dlInfo = ComputeDlOfdmaInfo ();
      m_dlInfo.txop = m_qosTxop[QosUtilsMapTidToAc (mpdu->GetHeader ().GetQosTid ())];
      // TODO check that RUs do not overlap?
    }
  else if (m_TxFormat == UL_OFDMA)
    {
      m_ulInfo = ComputeUlOfdmaInfo ();
    }
}

OfdmaTxFormat
OfdmaManager::GetTxFormat (void) const
{
  return m_TxFormat;
}

const OfdmaManager::DlOfdmaInfo&
OfdmaManager::GetDlOfdmaInfo (void) const
{
  NS_ABORT_MSG_IF (m_TxFormat != DL_OFDMA, "Next transmission is not DL OFDMA");
  return m_dlInfo;
}

const OfdmaManager::UlOfdmaInfo&
OfdmaManager::GetUlOfdmaInfo (void) const
{
  NS_ABORT_MSG_IF (m_TxFormat != UL_OFDMA, "Next transmission is not UL OFDMA");
  return m_ulInfo;
}

Ptr<WifiRemoteStationManager>
OfdmaManager::GetWifiRemoteStationManager (void) const
{
  return m_apMac->GetWifiRemoteStationManager ();
}

Time
OfdmaManager::GetResponseDuration (const MacLowTransmissionParameters& params, WifiTxVector txVector,
                                   CtrlTriggerHeader trigger) const
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (params.HasDlMuAckSequence ());

  Time response = Seconds (0);
  switch (params.GetDlMuAckSequenceType ())
    {
    case DlMuAckSequenceType::DL_SU_FORMAT:
      // Note that m_mpdu is not really used by the next call
      response = m_low->GetResponseDuration (params, txVector, m_mpdu);
      break;
    case DlMuAckSequenceType::DL_MU_BAR:
      {
        WifiMacHeader hdr;
        hdr.SetType (WIFI_MAC_CTL_TRIGGER);
        hdr.SetAddr1 (Mac48Address::GetBroadcast ());
        std::list<BlockAckReqType> barTypes;
        for (auto& sta : params.GetStationsSendBlockAckRequestTo ())
          {
            barTypes.push_back (params.GetBlockAckRequestType (sta));
          }
        uint32_t muBarSize = GetMuBarSize (barTypes);

        WifiTxVector muBarTxVector = GetWifiRemoteStationManager ()->GetRtsTxVector (hdr.GetAddr1 (), &hdr,
                                                                                     Create<Packet> (muBarSize));

        response = m_low->GetSifs () +
                   m_low->GetPhy ()->CalculateTxDuration (muBarSize, muBarTxVector,
                                                          m_low->GetPhy ()->GetFrequency ());
      }
    // do not break
    case DlMuAckSequenceType::DL_AGGREGATE_TF:
      {
        NS_ASSERT (trigger.IsMuBar ());
        response += m_low->GetSifs () +
                    WifiPhy::ConvertLSigLengthToHeTbPpduDuration (trigger.GetUlLength (),
                                                                  trigger.GetHeTbTxVector (trigger.begin ()->GetAid12 ()),
                                                                  m_low->GetPhy ()->GetFrequency ());
        break;
      }
    default:
      NS_FATAL_ERROR ("Unknown DL MU ack sequence");
      break;
    }
  NS_LOG_DEBUG ("Response duration: " << response.ToDouble (Time::US) << " us");
  return response;
}

void
OfdmaManager::SetTargetRssi (CtrlTriggerHeader& trigger) const
{
  NS_LOG_FUNCTION (this);

  trigger.SetApTxPower (static_cast<int8_t> (m_low->GetPhy ()->GetPowerDbm (GetWifiRemoteStationManager ()->GetDefaultTxPowerLevel ())));
  for (auto& userInfo : trigger)
    {
      const auto staList = m_apMac->GetStaList ();
      auto itAidAddr = staList.find (userInfo.GetAid12 ());
      NS_ASSERT (itAidAddr != staList.end ());
      int8_t rssi = static_cast<int8_t> (GetWifiRemoteStationManager ()->GetMostRecentRssi (itAidAddr->second));
      rssi = (rssi >= -20) ? -20 : ((rssi <= -110) ? -110 : rssi); //cap so as to keep within [-110; -20] dBm
      userInfo.SetUlTargetRssi (rssi);
    }
}

} //namespace ns3
