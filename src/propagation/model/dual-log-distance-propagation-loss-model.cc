/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Lucas de A. Cerqueira <lucas.cerqueira@poli.ufrj.br>
 * Universidade Federal do Rio de Janeiro (UFRJ)
 */

#include "ns3/propagation-loss-model.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/boolean.h"
#include "ns3/pointer.h"
#include <cmath>
#include <complex>
#include "string.h"
#include "dual-log-distance-propagation-loss-model.h"

#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("DualLogDistancePropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (DualLogDistancePropagationLossModel);

TypeId
DualLogDistancePropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::DualLogDistancePropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<DualLogDistancePropagationLossModel> ()
    .AddAttribute ("Distance0",
                   "Beginning of the first (near) distance field",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (
                     &DualLogDistancePropagationLossModel::m_distance0
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Distance1",
                   "Beginning of the second (far) distance field.",
                   DoubleValue (200.0),
                   MakeDoubleAccessor (
                     &DualLogDistancePropagationLossModel::m_distance1
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Exponent0",
                   "The exponent for the first field.",
                   DoubleValue (1.9),
                   MakeDoubleAccessor (
                     &DualLogDistancePropagationLossModel::m_exponent0
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Exponent1",
                   "The exponent for the second field.",
                   DoubleValue (3.8),
                   MakeDoubleAccessor (
                     &DualLogDistancePropagationLossModel::m_exponent1
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ReferenceLoss",
                   "The reference loss at distance d0 (dB). (Default is Friis\
                     at 1m with 5.9 GHz)",
                   DoubleValue (47.865),
                   MakeDoubleAccessor (
                     &DualLogDistancePropagationLossModel::m_referenceLoss
                   ),
                   MakeDoubleChecker<double> ())
  ;
  return tid;

}

DualLogDistancePropagationLossModel::DualLogDistancePropagationLossModel ()
{
}

double
DualLogDistancePropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                                     Ptr<MobilityModel> a,
                                                     Ptr<MobilityModel> b) const
{
  double distance = a->GetDistanceFrom (b);
  NS_ASSERT (distance >= 0);

  double pathLossDb;

  if (distance < m_distance0)
  {
    pathLossDb = 0;
  }
  else if (distance < m_distance1)
  {
    pathLossDb = m_referenceLoss
      + 10 * m_exponent0 * std::log10 (distance / m_distance0);
  }
  else
  {
    pathLossDb = m_referenceLoss
      + 10 * m_exponent0 * std::log10 (m_distance1 / m_distance0)
      + 10 * m_exponent1 * std::log10 (distance / m_distance1);
  }

  NS_LOG_DEBUG ("DualLogDistance distance=" << distance << "m, " <<
                "attenuation=" << pathLossDb << "dB");

  return txPowerDbm - pathLossDb;
}

int64_t
DualLogDistancePropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}
} // namespace ns3
