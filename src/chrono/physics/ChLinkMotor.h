// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHLINKMOTOR_H
#define CHLINKMOTOR_H


#include "chrono/physics/ChLinkMate.h"
#include "chrono/motion_functions/ChFunction.h"
#include "chrono/solver/ChVariablesGeneric.h"

namespace chrono {

/// Base class for all "motor" constraints between
/// two frames on two bodies. Look for children classes for 
/// specialized behaviors.

class ChApi ChLinkMotor : public ChLinkMateGeneric {

  public:
    ChLinkMotor() {}
    ChLinkMotor(const ChLinkMotor& other) : ChLinkMateGeneric(other) {}
    virtual ~ChLinkMotor() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotor* Clone() const override { return new ChLinkMotor(*this); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkMotor,0)



// -----------------------------------------------------------------------------



/// Base class for all linear "motor" constraints between
/// two frames on two bodies. Motors of this type assume that
/// the actuator is directed along X direction of the master frame.
/// Look for children classes for specialized behaviors.

class ChApi ChLinkMotorLinear : public ChLinkMotor {

    /// type of guide constraint 
    enum class GuideConstraint {
          FREE,
          PRISMATIC,
          SPHERICAL
      };

  protected:
    // aux data for optimization
    double mpos;
    double mpos_dt;
    double mpos_dtdt;

  public:
    ChLinkMotorLinear();
    ChLinkMotorLinear(const ChLinkMotorLinear& other);
    virtual ~ChLinkMotorLinear();

    /// "Virtual" copy constructor (covariant return type).
    //virtual ChLinkMotorLinear* Clone() const override { return new ChLinkMotorLinear(*this); }


    /// Sets which movements (of frame 1 respect to frame 2) are constrained. 
    /// By default, acts as a pure prismatic guide.
    /// Note that the x direction is the motorized one, and is never affected by 
    /// this option.
    void SetGuideConstraint(const GuideConstraint mconstraint);

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as a pure prismatic guide.
    /// Note that the x direction is the motorized one, and is never affected by 
    /// this option.
    void SetGuideConstraint(bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz); 

    /// Get the current actuator displacement [m], including error etc.
    virtual double GetActualPos() const {return mpos;}
    /// Get the current actuator speed [m/s], including error etc.
    virtual double GetActualPos_dt() const {return mpos_dt;}
    /// Get the current actuator acceleration [m/s^2], including error etc.
    virtual double GetActualPos_dtdt() const {return mpos_dtdt;}
    /// Get the current actuator reaction force [N]
    virtual double GetActualForce() const = 0;


    void Update(double mytime, bool update_assets) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorLinear,0)



// -----------------------------------------------------------------------------



/// A linear motor that enforces the position x(t) between
/// two frames on two bodies, using a rheonomic constraint.
/// The x(t) position of frame A sliding on x axis of frame B, 
/// is imposed via an exact function of time f(t), and an optional
/// offset:
///    x(t) = f(t) + offset
/// Note: no compliance is allowed, so if the actuator hits an undeformable 
/// obstacle it hits a pathological situation and the solver result
/// can be unstable/unpredictable.
/// Think at it as a servo drive with "infinitely stiff" control.
/// This type of motor is very easy to use, stable and efficient,
/// and should be used if the 'infinitely stiff' control assumption 
/// is a good approximation of what you simulate (ex very good and
/// reactive controllers).
/// By default it is initialized with linear ramp: df/dt= 1 m/s, use
/// SetMotionFunction() to change to other motion functions.

class ChApi ChLinkMotorLinearPosition : public ChLinkMotorLinear {

    std::shared_ptr<ChFunction> f_pos;
    double pos_offset;

  public:
    ChLinkMotorLinearPosition();
    ChLinkMotorLinearPosition(const ChLinkMotorLinearPosition& other);
    virtual ~ChLinkMotorLinearPosition();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorLinearPosition* Clone() const override { return new ChLinkMotorLinearPosition(*this); }


    /// Sets the position function f(t). It is a function of time. 
    /// Note that is must be C0 continuous. Better if C1 continuous too, otherwise
    /// it requires peaks in accelerations.
    void SetMotionFunction(const std::shared_ptr<ChFunction> mf) {f_pos = mf;}

    /// Gets the position function f(t).
    std::shared_ptr<ChFunction> GetMotionFunction() {return f_pos;}
    

    /// Get initial offset for f(t)=0. Position on x of the two axes
    /// will be x(t) = f(t) + offset.
    /// By default, offset = 0
    void SetMotionOffset(double mo) { pos_offset = mo; }

    /// Get initial offset for f(t)=0.
    double GetMotionOffset() { return pos_offset; }


    /// Get the current actuator reaction force [N]
    virtual double GetActualForce() const { return this->react_force.x();}


    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;


    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorLinearPosition,0)




// -----------------------------------------------------------------------------



/// A linear motor that enforces the speed v(t) between
/// two frames on two bodies, using a rheonomic constraint.
/// Note: no compliance is allowed, so if the actuator hits an undeformable 
/// obstacle it hits a pathological situation and the solver result
/// can be unstable/unpredictable.
/// Think at it as a servo drive with "infinitely stiff" control.
/// This type of motor is very easy to use, stable and efficient,
/// and should be used if the 'infinitely stiff' control assumption 
/// is a good approximation of what you simulate (ex very good and
/// reactive controllers).
/// By default it is initialized with constant speed: df/dt= 1 m/s, use
/// SetSpeedFunction() to change to other speed functions.

class ChApi ChLinkMotorLinearSpeed : public ChLinkMotorLinear {

  private:
    std::shared_ptr<ChFunction> f_speed;
    double pos_offset;

    ChVariablesGeneric variable;

    double aux_dt; // used for integrating speed, = pos
    double aux_dtdt;

    bool avoid_position_drift;

  public:
    ChLinkMotorLinearSpeed();
    ChLinkMotorLinearSpeed(const ChLinkMotorLinearSpeed& other);
    virtual ~ChLinkMotorLinearSpeed();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorLinearSpeed* Clone() const override { return new ChLinkMotorLinearSpeed(*this); }


    /// Sets the speed function v(t). It is a function of time. 
    /// Best if C0 continuous, otherwise it gives peaks in accelerations.
    void SetSpeedFunction(const std::shared_ptr<ChFunction> mf) {f_speed = mf;}

    /// Gets the speed function v(t).
    std::shared_ptr<ChFunction> GetSpeedFunction() {return f_speed;}
    

    /// Get initial offset, by default = 0.
    void SetMotionOffset(double mo) { pos_offset = mo; }

    /// Get initial offset.
    double GetMotionOffset() { return pos_offset; }

    /// Set if the constraint must avoid position drift. If true, it 
    /// means that the constraint is satisfied also at the position level,
    /// by integrating the velocity in a separate auxiliary state. Default, true.
    void SetAvoidPositionDrift(bool mb) {this->avoid_position_drift = mb;}

    /// Set if the constraint is in "avoid position drift" mode.
    bool GetAvoidPositionDrift() { return this->avoid_position_drift;}


    /// Get the current actuator reaction force [N]
    virtual double GetActualForce() const { return this->react_force.x();}


    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //

    virtual int GetDOF() override { return 1; } 

    virtual void IntStateGather(const unsigned int off_x, ChState& x, const unsigned int off_v, ChStateDelta& v, double& T) override;
    virtual void IntStateScatter(const unsigned int off_x, const ChState& x, const unsigned int off_v, const ChStateDelta& v, const double T) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off, ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) override;
    virtual void IntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R, const unsigned int off_L, const ChVectorDynamic<>& L,  const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,  ChStateDelta& v, const unsigned int off_L,  ChVectorDynamic<>& L) override;

    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //

    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;
    
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;


    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorLinearSpeed,0)


// -----------------------------------------------------------------------------



/// A linear motor that applies a force between
/// two frames on two bodies.
/// Differently from the ChLinkMotorLinearPosition and
/// ChLinkMotorLinearSpeed, this does not enforce precise
/// motion via constraint. 
/// Example of application:
/// - mimic a PID controlled system with 
///   some feedback (that is up to you too implement)
/// - force that is updated by a cosimulation
/// - force from a man-in-the-loop setpoint
/// Use SetForceFunction() to change to other speed function (by
/// default is no force), possibly introduce some custom ChFunction
/// of yours that is updated at each time step.

class ChApi ChLinkMotorLinearForce : public ChLinkMotorLinear {

    std::shared_ptr<ChFunction> f_force;

  public:
    ChLinkMotorLinearForce();
    ChLinkMotorLinearForce(const ChLinkMotorLinearForce& other);
    virtual ~ChLinkMotorLinearForce();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorLinearForce* Clone() const override { return new ChLinkMotorLinearForce(*this); }


    /// Sets the force function F(t). It is a function of time. 
    void SetForceFunction(const std::shared_ptr<ChFunction> mf) {f_force = mf;}

    /// Gets the force function F(t).
    std::shared_ptr<ChFunction> GetForceFunction() {return f_force;}
    

    /// Get the current actuator reaction force [N]
    virtual double GetActualForce() const { return this->f_force->Get_y(this->GetChTime());}

    
    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //
    virtual void ConstraintsFbLoadForces(double factor = 1) override;


    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorLinearForce,0)




// -----------------------------------------------------------------------------
//
// Rotational motors




/// Base class for all rotational "motor" constraints between
/// two frames on two bodies. Motors of this type assume that
/// the spindle is directed along Z direction of the master frame.
/// Look for children classes for specialized behaviors.

class ChApi ChLinkMotorRotation : public ChLinkMotor {

    /// type of guide constraint, exept the rotation constraint 
    enum class SpindleConstraint {
          FREE,
          REVOLUTE,
          CYLINDRICAL, 
          OLDHAM
      };

  protected:
    // aux data for optimization
    double mrot;
    double mrot_dt;
    double mrot_dtdt;

  public:
    ChLinkMotorRotation();
    ChLinkMotorRotation(const ChLinkMotorRotation& other);
    virtual ~ChLinkMotorRotation();

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as bearing, like a revolute joint.
    /// Note that the Z direction is the motorized one, and is never affected by 
    /// this option.
    void SetSpindleConstraint(const SpindleConstraint mconstraint);

    /// Sets which movements (of frame 1 respect to frame 2) are constrained.
    /// By default, acts as bearing, like a revolute joint.
    /// Note that the Z direction is the motorized one, and is never affected by 
    /// this option.
    void SetSpindleConstraint(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry); 

    /// Get the current actuator displacement [rad], including error etc.
    virtual double GetActualRot() const {return mrot;}
    /// Get the current actuator speed [rad/s], including error etc.
    virtual double GetActualRot_dt() const {return mrot_dt;}
    /// Get the current actuator acceleration [rad/s^2], including error etc.
    virtual double GetActualRot_dtdt() const {return mrot_dtdt;}
    /// Get the current actuator reaction torque [Nm]
    virtual double GetActualTorque() const = 0;


    void Update(double mytime, bool update_assets) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorRotation,0)



// -----------------------------------------------------------------------------



/// A motor that enforces the rotation angle r(t) between
/// two frames on two bodies, using a rheonomic constraint.
/// The r(t) angle of frame A rotating on Z axis of frame B, 
/// is imposed via an exact function of time f(t), and an optional
/// angle offset:
///    r(t) = f(t) + offset
/// Note: no compliance is allowed, so if the actuator hits an undeformable 
/// obstacle it hits a pathological situation and the solver result
/// can be unstable/unpredictable.
/// Think at it as a servo drive with "infinitely stiff" control.
/// This type of motor is very easy to use, stable and efficient,
/// and should be used if the 'infinitely stiff' control assumption 
/// is a good approximation of what you simulate (ex very good and
/// reactive controllers).
/// By default it is initialized with linear ramp: df/dt= 1 rad/s, use
/// SetAngleFunction() to change to other motion functions.

class ChApi ChLinkMotorRotationAngle : public ChLinkMotorRotation {

    std::shared_ptr<ChFunction> f_rot;
    double rot_offset;

  public:
    ChLinkMotorRotationAngle();
    ChLinkMotorRotationAngle(const ChLinkMotorRotationAngle& other);
    virtual ~ChLinkMotorRotationAngle();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorRotationAngle* Clone() const override { return new ChLinkMotorRotationAngle(*this); }


    /// Sets the rotation angle function f(t), in [rad]. It is a function of time. 
    /// Note that is must be C0 continuous. Better if C1 continuous too, otherwise
    /// it requires peaks in accelerations.
    void SetAngleFunction(const std::shared_ptr<ChFunction> mf) {f_rot = mf;}

    /// Gets the rotation angle function f(t).
    std::shared_ptr<ChFunction> GetAngleFunction() {return f_rot;}
    

    /// Get initial angle offset for f(t)=0, in [rad]. Rotation on Z of the two axes
    /// will be r(t) = f(t) + offset.
    /// By default, offset = 0
    void SetMotionOffset(double mo) { rot_offset = mo; }

    /// Get initial offset for f(t)=0, in [rad]
    double GetMotionOffset() { return rot_offset; }


    /// Get the current actuator reaction torque [Nm]
    virtual double GetActualTorque() const { return this->react_torque.x();}


    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //
    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;


    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorRotationAngle,0)




// -----------------------------------------------------------------------------



/// A motor that enforces the angular speed w(t) between
/// two frames on two bodies, using a rheonomic constraint.
/// Note: no compliance is allowed, so if the actuator hits an undeformable 
/// obstacle it hits a pathological situation and the solver result
/// can be unstable/unpredictable.
/// Think at it as a servo drive with "infinitely stiff" control.
/// This type of motor is very easy to use, stable and efficient,
/// and should be used if the 'infinitely stiff' control assumption 
/// is a good approximation of what you simulate (ex very good and
/// reactive controllers).
/// By default it is initialized with constant angular speed: df/dt= 1 rad/s, use
/// SetSpeedFunction() to change to other speed functions.

class ChApi ChLinkMotorRotationSpeed : public ChLinkMotorRotation {

  private:
    std::shared_ptr<ChFunction> f_speed;
    double rot_offset;

    ChVariablesGeneric variable;

    double aux_dt; // used for integrating speed, = angle
    double aux_dtdt;

    bool avoid_angle_drift;

  public:
    ChLinkMotorRotationSpeed();
    ChLinkMotorRotationSpeed(const ChLinkMotorRotationSpeed& other);
    virtual ~ChLinkMotorRotationSpeed();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorRotationSpeed* Clone() const override { return new ChLinkMotorRotationSpeed(*this); }


    /// Sets the angular speed function w(t), in [rad/s]. It is a function of time. 
    /// Best if C0 continuous, otherwise it gives peaks in accelerations.
    void SetSpeedFunction(const std::shared_ptr<ChFunction> mf) {f_speed = mf;}

    /// Gets the speed function w(t). In [rad/s].
    std::shared_ptr<ChFunction> GetSpeedFunction() {return f_speed;}
    

    /// Get initial offset, in [rad]. By default = 0.
    void SetAngleOffset(double mo) { rot_offset = mo; }

    /// Get initial offset, in [rad].
    double GetAngleOffset() { return rot_offset; }

    /// Set if the constraint must avoid angular drift. If true, it 
    /// means that the constraint is satisfied also at the rotation level,
    /// by integrating the velocity in a separate auxiliary state. Default, true.
    void SetAvoidAngleDrift(bool mb) {this->avoid_angle_drift = mb;}

    /// Set if the constraint is in "avoid angle drift" mode.
    bool GetAvoidAngleDrift() { return this->avoid_angle_drift;}


    /// Get the current actuator reaction torque [Nm]
    virtual double GetActualTorque() const { return this->react_torque.x();}


    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //

    virtual int GetDOF() override { return 1; } 

    virtual void IntStateGather(const unsigned int off_x, ChState& x, const unsigned int off_v, ChStateDelta& v, double& T) override;
    virtual void IntStateScatter(const unsigned int off_x, const ChState& x, const unsigned int off_v, const ChStateDelta& v, const double T) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off, ChVectorDynamic<>& R, const ChVectorDynamic<>& w, const double c) override;
    virtual void IntToDescriptor(const unsigned int off_v, const ChStateDelta& v, const ChVectorDynamic<>& R, const unsigned int off_L, const ChVectorDynamic<>& L,  const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,  ChStateDelta& v, const unsigned int off_L,  ChVectorDynamic<>& L) override;

    virtual void IntLoadConstraint_Ct(const unsigned int off, ChVectorDynamic<>& Qc, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //

    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;
    
    virtual void ConstraintsBiLoad_Ct(double factor = 1) override;


    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorRotationSpeed,0)


// -----------------------------------------------------------------------------



/// A motor that applies a torque between
/// two frames on two bodies.
/// Differently from the ChLinkMotorRotationAngle and
/// ChLinkMotorRotationSpeed, this does not enforce precise
/// motion via constraint. 
/// Example of application:
/// - mimic a PID controlled system with 
///   some feedback (that is up to you too implement)
/// - force that is updated by a cosimulation
/// - force from a man-in-the-loop setpoint
/// Use SetTorqueFunction() to change to other speed function (by
/// default is no force), possibly introduce some custom ChFunction
/// of yours that is updated at each time step.

class ChApi ChLinkMotorRotationTorque : public ChLinkMotorRotation {

    std::shared_ptr<ChFunction> f_torque;

  public:
    ChLinkMotorRotationTorque();
    ChLinkMotorRotationTorque(const ChLinkMotorRotationTorque& other);
    virtual ~ChLinkMotorRotationTorque();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorRotationTorque* Clone() const override { return new ChLinkMotorRotationTorque(*this); }


    /// Sets the torque function T(t). In [Nm]. It is a function of time. 
    void SetTorqueFunction(const std::shared_ptr<ChFunction> mf) {f_torque = mf;}

    /// Gets the torque function F(t).
    std::shared_ptr<ChFunction> GetTorqueFunction() {return f_torque;}
    

    /// Get the current actuator reaction torque [Nm]
    virtual double GetActualTorque() const { return this->f_torque->Get_y(this->GetChTime());}

    
    void Update(double mytime, bool update_assets) override;

    //
    // STATE FUNCTIONS
    //
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    //
    // SOLVER INTERFACE (OLD)
    //
    virtual void ConstraintsFbLoadForces(double factor = 1) override;


    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

};

CH_CLASS_VERSION(ChLinkMotorRotationTorque,0)







}  // end namespace chrono

#endif
