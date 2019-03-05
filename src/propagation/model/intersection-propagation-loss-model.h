/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Lucas de A. Cerqueira <lucas.cerqueira@poli.ufrj.br>
 * Universidade Federal do Rio de Janeiro (UFRJ)
*/

#ifndef INTERSECTION_PROPAGATION_LOSS_MODEL_H
#define INTERSECTION_PROPAGATION_LOSS_MODEL_H

#include "string.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "ns3/propagation-loss-model.h"
#include <map>

namespace ns3 {

   class IntersectionPropagationLossModel : public PropagationLossModel
   {
   public:
     /**
      * \brief Get the type ID.
      * \return the object TypeId
      */
     static TypeId GetTypeId (void);
     IntersectionPropagationLossModel ();

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
      * \param intersec_center
      *
      * Set the intersection center position.
      */
     void SetIntersection (Vector2D intersec_center);

     /**
      * \returns the intersection center position
      */
     Vector2D GetIntersection (void) const;


   private:
     /**
      * \brief Copy constructor
      *
      * Defined and unimplemented to avoid misuse
      */
     IntersectionPropagationLossModel (
       const IntersectionPropagationLossModel&
     );
     /**
      * \brief Copy constructor
      *
      * Defined and unimplemented to avoid misuse
      * \returns
      */
     IntersectionPropagationLossModel& operator= (
       const IntersectionPropagationLossModel&
     );

     virtual double DoCalcRxPower (double txPowerDbm,
                                   Ptr<MobilityModel> a,
                                   Ptr<MobilityModel> b) const;
     virtual int64_t DoAssignStreams (int64_t stream);

     double m_lambda;        //!< the carrier wavelength
     double m_frequency;     //!< the carrier frequency

     bool m_intersec_set = false; //!< Whether intersection was set
     Vector2D m_intersec_center;  //!< Intersection center coordinates

     double m_min_distance; //!< Minimum distance to the intersection center
     double m_break_distance; //!< Break even distance.

     double m_rx_street_width; //!< Receiver street width
     double m_tx_wall_distance; //!< Distance of transmitter to wall

     bool m_is_suburban;
   };
} /* namespace ns3*/

#endif /* INTERSECTION_PROPAGATION_LOSS_MODEL_H */
