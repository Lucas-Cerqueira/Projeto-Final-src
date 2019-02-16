/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Lucas de A. Cerqueira <lucas.cerqueira@poli.ufrj.br>
 * Universidade Federal do Rio de Janeiro (UFRJ)
 */

#ifndef KUNISCH_TWORAY_PROPAGATION_LOSS_MODEL_H
#define KUNISCH_TWORAY_PROPAGATION_LOSS_MODEL_H

#include "string.h"
#include "ns3/nstime.h"
#include "ns3/propagation-loss-model.h"
#include <map>

namespace ns3 {

  class KunischTwoRayPropagationLossModel : public PropagationLossModel
  {
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId (void);
    KunischTwoRayPropagationLossModel ();

    /**
     * \param frequency (Hz)
     *
     * Set the carrier frequency used in the TwoRay model
     * calculation.
     */
    void SetFrequency (double frequency);

    /**
     * \returns the current frequency (Hz)
     */
    double GetFrequency (void) const;

    /**
     * \param minDistance the minimum distance
     *
     * Below this distance, the txpower is returned
     * unmodified as the rxpower.
     */
    void SetMinDistance (double minDistance);
    /**
     * \returns the minimum distance.
     */
    double GetMinDistance (void) const;

    /**
     * \returns the current system loss (dimension-less)
     */
    double GetSystemLoss (void) const;

    /**
     * \param noiseStd the noise standard deviation
     *
     * Sets the AWGN standard deviation
     */
    void SetNoiseStd (double noiseStd);

    /**
     * \returns the AWGN standard deviation
     */
    double GetNoiseStd (void) const;

  private:
    /**
     * \brief Copy constructor
     *
     * Defined and unimplemented to avoid misuse
     */
    KunischTwoRayPropagationLossModel (
      const KunischTwoRayPropagationLossModel &
    );
    /**
     * \brief Copy constructor
     *
     * Defined and unimplemented to avoid misuse
     * \returns
     */
    KunischTwoRayPropagationLossModel & operator = (
      const KunischTwoRayPropagationLossModel &
    );

    virtual double DoCalcRxPower (double txPowerDbm,
                                  Ptr<MobilityModel> a,
                                  Ptr<MobilityModel> b) const;
    virtual int64_t DoAssignStreams (int64_t stream);

    /**
     * Transforms a Dbm value to Watt
     * \param dbm the Dbm value
     * \return the Watts
     */
    double DbmToW (double dbm) const;

    /**
     * Transforms a Watt value to Dbm
     * \param w the Watt value
     * \return the Dbm
     */
    double DbmFromW (double w) const;

    double m_lambda;        //!< the carrier wavelength
    double m_frequency;     //!< the carrier frequency
    double m_minDistance;   //!< minimum distance for the model

    double m_coefficientMag; //!< reflection coefficient magnitude
    double m_coefficientPhase; //!< reflection coefficient phase
    double m_baseGain; //!< constant base channel gain
  };

} /* namespace ns3*/

#endif /* KUNISCH_TWORAY_PROPAGATION_LOSS_MODEL_H */
