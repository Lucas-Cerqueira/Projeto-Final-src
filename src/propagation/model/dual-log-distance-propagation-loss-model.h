/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Lucas de A. Cerqueira <lucas.cerqueira@poli.ufrj.br>
 * Universidade Federal do Rio de Janeiro (UFRJ)
 */

#ifndef DUAL_LOG_DISTANCE_PROPAGATION_LOSS_MODEL_H
#define DUAL_LOG_DISTANCE_PROPAGATION_LOSS_MODEL_H

#include "string.h"
#include "ns3/nstime.h"
#include "ns3/propagation-loss-model.h"
#include <map>

namespace ns3 {

   class DualLogDistancePropagationLossModel : public PropagationLossModel
   {
   public:
     /**
      * \brief Get the type ID.
      * \return the object TypeId
      */
     static TypeId GetTypeId (void);
     DualLogDistancePropagationLossModel ();

   private:
     /**
      * \brief Copy constructor
      *
      * Defined and unimplemented to avoid misuse
      */
     DualLogDistancePropagationLossModel (
       const DualLogDistancePropagationLossModel&
     );
     /**
      * \brief Copy constructor
      *
      * Defined and unimplemented to avoid misuse
      * \returns
      */
     DualLogDistancePropagationLossModel& operator= (
       const DualLogDistancePropagationLossModel&
     );

     virtual double DoCalcRxPower (double txPowerDbm,
                                   Ptr<MobilityModel> a,
                                   Ptr<MobilityModel> b) const;
     virtual int64_t DoAssignStreams (int64_t stream);

     double m_distance0; //!< Beginning of the first (near) distance field
     double m_distance1; //!< Beginning of the second (far) distance field.

     double m_exponent0; //!< The exponent for the first field.
     double m_exponent1; //!< The exponent for the second field.

     double m_referenceLoss; //!< The reference loss at distance d0 (dB).
   };
} /* namespace ns3*/

#endif /* DUAL_LOG_DISTANCE_PROPAGATION_LOSS_MODEL_H */
