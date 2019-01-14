/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007,2008, 2009 INRIA, UDcast
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
 * Author: Mohamed Amine Ismail <amine.ismail@sophia.inria.fr>
 *                              <amine.ismail@udcast.com>
 */

#ifndef KUNISCH_TWORAY_PROPAGATION_LOSS_MODEL_H
#define KUNISCH_TWORAY_PROPAGATION_LOSS_MODEL_H

#include "string.h"
#include "ns3/nstime.h"
#include "ns3/propagation-loss-model.h"
#include <map>

namespace ns3 {

  /**
   * \ingroup propagation
   *
   * \brief a log distance propagation model for LOS and OLOS for VANET.
   *
   * Developed by VOLVO...
   *
   * where:
   *  - \f$ n \f$ : the path loss distance exponent
   *  - \f$ d_0 \f$ : reference distance (m)
   *  - \f$ L_0 \f$ : path loss at reference distance (dB)
   *  - \f$ d \f$ : distance (m)
   *  - \f$ L \f$ : path loss (dB)
   *
   * When the path loss is requested at a distance smaller than
   * the reference distance, the tx power is returned.
   *
   */
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
     * \param heightAboveZ the model antenna height above the node's Z coordinate
     *
     * Set the model antenna height above the node's Z coordinate
     */
    void SetHeightAboveZ (double heightAboveZ);

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
    KunischTwoRayPropagationLossModel (const KunischTwoRayPropagationLossModel &);
    /**
     * \brief Copy constructor
     *
     * Defined and unimplemented to avoid misuse
     * \returns
     */
    KunischTwoRayPropagationLossModel & operator = (const KunischTwoRayPropagationLossModel &);

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
    double m_heightAboveZ;  //!< antenna height above the node's Z coordinate

    double m_coefficientMag; //!< reflection coefficient magnitude
    double m_coefficientPhase; //!< reflection coefficient phase
    double m_baseGain; //!< constant base channel gain
  };

} /* namespace ns3*/

#endif /* KUNISCH_TWORAY_PROPAGATION_LOSS_MODEL_H */
