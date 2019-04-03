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
#include "wifi-ack-policy-selector.h"
#include <set>
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("WifiAckPolicySelector");

NS_OBJECT_ENSURE_REGISTERED (WifiAckPolicySelector);

TypeId WifiAckPolicySelector::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::WifiAckPolicySelector")
    .SetParent<Object> ()
    .SetGroupName("Wifi")
  ;
  return tid;
}

WifiAckPolicySelector::~WifiAckPolicySelector ()
{
  NS_LOG_FUNCTION (this);
}

void
WifiAckPolicySelector::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_qosTxop = 0;
}

void
WifiAckPolicySelector::SetQosTxop (Ptr<QosTxop> qosTxop)
{
  NS_LOG_FUNCTION (this << qosTxop);
  m_qosTxop = qosTxop;
}

Ptr<QosTxop>
WifiAckPolicySelector::GetQosTxop (void) const
{
  return m_qosTxop;
}

void
WifiAckPolicySelector::SetAckPolicy (std::map <uint16_t, Ptr<WifiPsdu>> psduMap,
                                     const MacLowTransmissionParameters & params)
{
  NS_LOG_FUNCTION (params);

  for (auto& psdu : psduMap)
    {
      std::set<uint8_t> tids = psdu.second->GetTids ();
      NS_ASSERT (tids.size () == 1);
      uint8_t tid = *tids.begin ();

      /* SU PPDU or DL MU PPDU not requiring acknowledgment */
      if (!params.HasDlMuAckSequence () && !params.HasUlMuAckSequence ())
        {
          if (params.MustWaitNormalAck () || params.MustWaitBlockAck ())
            {
              // Normal Ack or Implicit Block Ack Request policy
              psdu.second->SetAckPolicyForTid (tid, WifiMacHeader::NORMAL_ACK);
            }
          else
            {
              // Block Ack policy
              psdu.second->SetAckPolicyForTid (tid, WifiMacHeader::BLOCK_ACK);
            }
        }

      /* DL MU PPDU requiring acknowledgment */
      if (params.HasDlMuAckSequence ())
        {
          Mac48Address receiver = psdu.second->GetAddr1 ();

          if (params.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_SU_FORMAT)
            {
              // sequence of BAR+BA frames
              auto list = params.GetStationsReceiveAckFrom ();
              if (std::find (list.begin (), list.end (), receiver) != list.end ())
                {
                  // Normal Ack or Implicit Block Ack Request policy
                  psdu.second->SetAckPolicyForTid (tid, WifiMacHeader::NORMAL_ACK);
                  continue;
                }

              list = params.GetStationsReceiveBlockAckFrom ();
              if (std::find (list.begin (), list.end (), receiver) != list.end ())
                {
                  // Normal Ack or Implicit Block Ack Request policy
                  psdu.second->SetAckPolicyForTid (tid, WifiMacHeader::NORMAL_ACK);
                  continue;
                }

              // Block Ack policy
              psdu.second->SetAckPolicyForTid (tid, WifiMacHeader::BLOCK_ACK);
            }
          else if (params.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_MU_BAR)
            {
              // stations need to wait for a MU-BAR
              psdu.second->SetAckPolicyForTid (tid, WifiMacHeader::BLOCK_ACK);
            }
          else if (params.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF)
            {
              // stations send a Block Ack in response to the Trigger frame
              // No explicit acknowledgment or HE TB PPDU (HTP) Ack policy
              psdu.second->SetAckPolicyForTid (tid, WifiMacHeader::NO_EXPLICIT_ACK);
            }
        }
    }
}

} //namespace ns3
