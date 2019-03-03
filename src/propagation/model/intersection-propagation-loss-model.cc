/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Lucas de A. Cerqueira <lucas.cerqueira@poli.ufrj.br>
 * Universidade Federal do Rio de Janeiro (UFRJ)
 */

#include "ns3/propagation-loss-model.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/attribute.h"
#include "ns3/vector.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/boolean.h"
#include "ns3/pointer.h"
#include <cmath>
#include <complex>
#include "string.h"
#include "intersection-propagation-loss-model.h"

#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("IntersectionPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (IntersectionPropagationLossModel);

TypeId
IntersectionPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::IntersectionPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<IntersectionPropagationLossModel> ()
    .AddAttribute ("IntersectionCenter",
                   "Intersection center coordinates (x, y).",
                   EmptyAttributeValue (),
                   MakeVector2DAccessor (
                     &IntersectionPropagationLossModel::SetIntersection,
                     &IntersectionPropagationLossModel::GetIntersection
                   ),
                   MakeVector2DChecker ())
    .AddAttribute ("Frequency",
                   "The carrier frequency (in Hz) at which\
                   propagation occurs (default is 5.15 GHz).",
                   DoubleValue (5.9e9),
                   MakeDoubleAccessor (
                     &IntersectionPropagationLossModel::SetFrequency,
                     &IntersectionPropagationLossModel::GetFrequency
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MinDistance",
                   "The receiver minimum distance to the intersection\
                   in meters (default is 10 m).",
                   DoubleValue (10),
                   MakeDoubleAccessor (
                     &IntersectionPropagationLossModel::m_min_distance
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BreakDistance",
                   "The break event distance in meters\
                   (default is Fresnel distance).",
                   DoubleValue (-1),
                   MakeDoubleAccessor (
                     &IntersectionPropagationLossModel::m_break_distance
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxStreetWidth",
                   "Receiver street width in meters (default is 30 m).",
                   DoubleValue (30),
                   MakeDoubleAccessor (
                     &IntersectionPropagationLossModel::m_rx_street_width
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("TxWallDistance",
                   "Distance of transmitter to wall in meters \
                   (default is 15 m).",
                   DoubleValue (-1),
                   MakeDoubleAccessor (
                     &IntersectionPropagationLossModel::m_tx_wall_distance
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("IsSuburban",
                   "Flag to indicate whether it is a suburban \
                   environment or not (default is false).",
                   BooleanValue (false),
                   MakeBooleanAccessor (
                     &IntersectionPropagationLossModel::m_is_suburban
                   ),
                   MakeBooleanChecker ())
  ;
  return tid;

}

void
IntersectionPropagationLossModel::SetFrequency (double frequency)
{
  m_frequency = frequency;
  static const double C = 299792458.0; // speed of light in vacuum
  m_lambda = C / frequency;
}

double
IntersectionPropagationLossModel::GetFrequency (void) const
{
  return m_frequency;
}

void
IntersectionPropagationLossModel::SetIntersection (
  Vector2D intersec_center
)
{
  m_intersec_center = intersec_center;
  m_intersec_set = true;
}

Vector2D
IntersectionPropagationLossModel::GetIntersection (void) const
{
  return m_intersec_center;
}

IntersectionPropagationLossModel::IntersectionPropagationLossModel ()
{
  m_intersec_set = false;
}

double
IntersectionPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                                     Ptr<MobilityModel> a,
                                                     Ptr<MobilityModel> b) const
{
  NS_ASSERT_MSG (
    m_intersec_set == true,
    "The attribute IntersectionCenter must be set."
  );

  Vector3D txPos = a->GetPosition ();
  Vector3D rxPos = b->GetPosition ();

  double txHeight = txPos.z;
  double rxHeight = rxPos.z;

  double txDistance = CalculateDistance (
    Vector2D (txPos.x, txPos.y),
    m_intersec_center
  );

  double rxDistance = CalculateDistance (
    Vector2D (rxPos.x, rxPos.y),
    m_intersec_center
  );

  double tx_wall_distance;
  double break_distance;
  double pathLossDb;

  if (m_break_distance == -1) // Break even distance not set
  {
    // If it was not set, use Fresnel zone distance
    break_distance = 4 * txHeight * rxHeight / m_lambda;
  }
  else
    break_distance = m_break_distance;

  if (m_tx_wall_distance == -1) // Tx distance to wall not set
  {
    // If it was not set, use half of the receiver street width
    tx_wall_distance = m_rx_street_width / 2.0;
  }
  else
    tx_wall_distance = m_tx_wall_distance;

  NS_LOG_DEBUG ("txDistance=" << txDistance << "m, " <<
                "rxDistance=" << rxDistance << "m, " <<
                "inters_center=" << m_intersec_center << ", " <<
                "txPos=" << txPos << ", " <<
                "rxPos=" << rxPos << ", " <<
                "x_t=" << tx_wall_distance << "m, " <<
                "w_r=" << m_rx_street_width << "m, " <<
                "d_b=" << break_distance << "m");

  if (rxDistance < m_min_distance)
  {
    pathLossDb = 0;
  }
  else
  {
    double geometryFactor = std::pow (
      tx_wall_distance * m_rx_street_width,
      0.81
    );

    double distanceFactor = std::pow (txDistance, 0.957) * rxDistance;

    if (rxDistance > break_distance)
      distanceFactor *= (rxDistance / break_distance);

    pathLossDb = 3.75 + 26.9 * std::log10 (
      distanceFactor / geometryFactor * 4.0 * M_PI / m_lambda
    ) + 2.94 * (int) m_is_suburban;

    pathLossDb += 1.75; // system loss
  }

  NS_LOG_DEBUG ("propagation: rxDistance=" << rxDistance << "m, " <<
                "pathLoss=" << pathLossDb << "dB, " <<
                "rxPower=" << (20-pathLossDb) << "dBm");

  return txPowerDbm - pathLossDb;
}

int64_t
IntersectionPropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}
} // namespace ns3
