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
      NS_ABORT_MSG_IF (m_dlInfo.staInfo.empty (), "No DL OFDMA info returned by OfdmaManager");
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

} //namespace ns3
