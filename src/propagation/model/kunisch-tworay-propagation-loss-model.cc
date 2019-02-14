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
#include "kunisch-tworay-propagation-loss-model.h"

#include <fstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("KunischTwoRayPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (KunischTwoRayPropagationLossModel);

TypeId
KunischTwoRayPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::KunischTwoRayPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<KunischTwoRayPropagationLossModel> ()
    .AddAttribute ("Frequency",
                   "The carrier frequency (in Hz) at which propagation occurs \
                   (default is 5.15 GHz).",
                   DoubleValue (5.9e9),
                   MakeDoubleAccessor (
                     &KunischTwoRayPropagationLossModel::SetFrequency,
                     &KunischTwoRayPropagationLossModel::GetFrequency
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MinDistance",
                   "The distance under which the propagation model uses \
                   Friis model (m)",
                   DoubleValue (32),
                   MakeDoubleAccessor (
                     &KunischTwoRayPropagationLossModel::SetMinDistance,
                     &KunischTwoRayPropagationLossModel::GetMinDistance
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("HeightAboveZ",
                   "The height of the antenna (m) above the node's Z coordinate\
                   (default is 1.5 m)",
                   DoubleValue (1.81),
                   MakeDoubleAccessor (
                     &KunischTwoRayPropagationLossModel::m_heightAboveZ
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("BaseGain",
                   "The constant base gain of the channel in dB \
                   (default is -9.0 dB)",
                   DoubleValue (-9.0),
                   MakeDoubleAccessor (
                     &KunischTwoRayPropagationLossModel::m_baseGain
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ReflectionCoefficientMag",
                   "The magnitude of the complex reflection coefficient \
                   (default is 0.353)",
                   DoubleValue (0.353),
                   MakeDoubleAccessor (
                     &KunischTwoRayPropagationLossModel::m_coefficientMag
                   ),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ReflectionCoefficientPhase",
                   "The phase of the complex reflection coefficient in degrees \
                   (default is -154 degrees)",
                   DoubleValue (-154.0),
                   MakeDoubleAccessor (
                     &KunischTwoRayPropagationLossModel::m_coefficientPhase
                   ),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

KunischTwoRayPropagationLossModel::KunischTwoRayPropagationLossModel ()
{
}

void
KunischTwoRayPropagationLossModel::SetMinDistance (double minDistance)
{
  m_minDistance = minDistance;
}

double
KunischTwoRayPropagationLossModel::GetMinDistance (void) const
{
  return m_minDistance;
}

void
KunischTwoRayPropagationLossModel::SetHeightAboveZ (double heightAboveZ)
{
  m_heightAboveZ = heightAboveZ;
}

void
KunischTwoRayPropagationLossModel::SetFrequency (double frequency)
{
  m_frequency = frequency;
  static const double C = 299792458.0; // speed of light in vacuum
  m_lambda = C / frequency;
}

double
KunischTwoRayPropagationLossModel::GetFrequency (void) const
{
  return m_frequency;
}

double
KunischTwoRayPropagationLossModel::DbmToW (double dbm) const
{
  double mw = std::pow (10.0,dbm / 10.0);
  return mw / 1000.0;
}

double
KunischTwoRayPropagationLossModel::DbmFromW (double w) const
{
  double dbm = std::log10 (w * 1000.0) * 10.0;
  return dbm;
}

double
KunischTwoRayPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                                 Ptr<MobilityModel> a,
                                                 Ptr<MobilityModel> b) const
{

  double distance = a->GetDistanceFrom (b);

  // Set the height of the Tx and Rx antennas
  // double txAntHeight = a->GetPosition ().z + m_heightAboveZ;
  // double rxAntHeight = b->GetPosition ().z + m_heightAboveZ;

  // double reflectedDistance = std::sqrt(std::pow(txAntHeight + rxAntHeight, 2) +
  //                                      std::pow(distance, 2));

  double reflectedDistance = std::sqrt(std::pow(2 * m_heightAboveZ, 2) +
                                       std::pow(distance, 2));

  if (distance <= m_minDistance)
    {
      // Use Friis model
      double numerator = m_lambda * m_lambda;
      double tmp = M_PI * distance;
      double denominator = 16 * tmp * tmp;
      double pr = 10 * std::log10 (numerator / denominator) + m_baseGain;
      NS_LOG_DEBUG ("Receiver closer than minDistance (" << m_minDistance <<"m)\
                    for Two_ray path; using Friis");
      NS_LOG_DEBUG ("distance=" << distance << "m, attenuation coefficient=" <<
                    pr << "dB");
      return txPowerDbm + pr;
    }

  double k = (2.0 * M_PI) / m_lambda;

  const std::complex<double> j(0, 1);

  double phaseInRadians = (m_coefficientPhase * M_PI) / 180.0;
  std::complex<double> groundCoefficient = std::polar(m_coefficientMag,
                                                      phaseInRadians);

  std::complex<double>
  complexFactor = (std::exp((-j) * k * distance) / distance) +
                  (groundCoefficient * std::exp((-j) * k * reflectedDistance) /
                  reflectedDistance);

  double
  channelGain = m_baseGain + 20.0 * (
    std::log10 ((m_lambda / (4.0 * M_PI)) * std::abs(complexFactor))
  );

  NS_LOG_DEBUG ("distance=" << distance << "m, txPower=" << txPowerDbm <<
                "dBm, channelGain=" << channelGain << "dB");

  return txPowerDbm + channelGain;
}

int64_t
KunischTwoRayPropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}
} // namespace ns3
