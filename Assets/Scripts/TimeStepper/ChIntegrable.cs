using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace chrono
{

    // -----------------------------------------------------------------------------

    /// Interface class for all objects that support time integration.
    /// Derived concrete classes can use time integrators for the ChTimestepper hierarchy.
    public interface ChIntegrable
    {

        /// Return the number of coordinates in the state Y.
        int GetNcoords_y();

        /// Return the number of coordinates in the state increment.
        /// This is a base implementation that works in many cases where dim(Y) = dim(dy),
        /// but it can be overridden in the case that y contains quaternions for rotations
        /// rather than simple y+dy
        int GetNcoords_dy();

        /// Return the number of lagrangian multipliers (constraints).
        /// By default returns 0.
        int GetNconstr();

        /// Gather system state in specified array.
        /// Optionally, they will copy system private state, if any, to Y.
        void StateGather(ref ChState y, ref double T);

        /// Scatter the states from the provided array to the system.
        /// This function is called by time integrators every time they modify the Y state.
        /// In some cases, the ChIntegrable object might contain dependent data structures
        /// that might need an update at each change of Y. If so, this function must be overridden.
        void StateScatter(ChState y, double T);

        /// Gather from the system the state derivatives in specified array.
        /// Optional: the integrable object might contain last computed state derivative, some integrators might reuse it.
        void StateGatherDerivative(ref ChStateDelta Dydt);

        /// Scatter the state derivatives from the provided array to the system.
        /// Optional: the integrable object might need to store last computed state derivative, ex. for plotting etc.
        void StateScatterDerivative(ChStateDelta Dydt);

        /// Gather from the system the Lagrange multipliers in specified array.
        /// Optional: the integrable object might contain Lagrange multipliers (reaction in constraints)
        void StateGatherReactions(ref ChVectorDynamic<double> L);

        /// Scatter the Lagrange multipliers from the provided array to the system.
        /// Optional: the integrable object might contain Lagrange multipliers (reaction in constraints)
        void StateScatterReactions(ChVectorDynamic<double> L);

        /// Solve for state derivatives: dy/dt = f(y,t).
        /// Given current state y , computes the state derivative dy/dt and Lagrange multipliers L (if any).
        /// NOTE: some solvers (ex in DVI) cannot compute a classical derivative dy/dt when v is a function of
        /// bounded variation, and f or L are distributions (e.g., when there are impulses and discontinuities),
        /// so they compute a finite Dy through a finite dt. This is the reason why this function has an optional
        /// parameter dt. In a DVI setting, one computes Dy, and returns Dy*(1/dt) here in Dydt parameter; if the
        /// original Dy has to be known, just multiply Dydt*dt. The same for impulses: a DVI would compute
        /// impulses I, and return L=I*(1/dt).
        /// NOTES:
        ///    - derived classes must take care of calling StateScatter(y,T) before computing Dy, only if
        ///      force_state_scatter = true (otherwise it is assumed state is already in sync)
        ///    - derived classes must take care of resizing Dy and L if needed.
        ///
        /// This function must return true if successful and false otherwise.
        abstract bool StateSolve(ref ChStateDelta Dydt,              //< result: computed Dydt
                            ref ChVectorDynamic<double> L,            //< result: computed lagrangian multipliers, if any
                            ChState y,                                  //< current state y
                            double T,                                        //< current time T
                            double dt,                                  //< timestep (if needed, ex. in DVI)
                            bool force_state_scatter = true             //< if false, y and T are not scattered to the system
                            );

        /// Increment state array: y_new = y + Dy.
        /// This is a base implementation that works in many cases, but it can be overridden
        /// in the case that y contains quaternions for rotations, in which case rot. exponential is needed
        /// instead of simply doing y+Dy.
        /// NOTE: the system is not updated automatically after the state increment, so one might
        /// need to call StateScatter().
        void StateIncrement(ref ChState y_new,         //< resulting y_new = y + Dy
                                ChState y,       //< initial state y
                                ChStateDelta Dy  //< state increment Dy
                                );

        //
        // Functions required by implicit integration schemes
        //

        /// Assuming an explicit ODE
        ///    H*dy/dt = F(y,t)
        /// or an explicit DAE
        ///    H*dy/dt = F(y,t) + Cq*L
        ///     C(y,t) = 0
        /// this function must compute the state increment as required for a Newton iteration
        /// within an implicit integration scheme.
        /// For an ODE:
        ///  Dy = [ c_a*H + c_b*dF/dy ]^-1 * R
        ///  Dy = [ G ]^-1 * R
        /// For a DAE with constraints:
        ///  |Dy| = [ G   Cq' ]^-1 * | R |
        ///  |DL|   [ Cq  0   ]      | Qc|
        /// where R is a given residual and dF/dy is the Jacobian of F.
        ///
        /// This function must return true if successful and false otherwise.
        abstract bool StateSolveCorrection(
            ref ChStateDelta Dy,                 //< result: computed Dy
            ref ChVectorDynamic<double> L,             //< result: computed lagrangian multipliers, if any
            ChVectorDynamic<double> R,       //< the R residual
            ChVectorDynamic<double> Qc,      //< the Qc residual
            double a,                   //< the factor in c_a*H
            double b,                   //< the factor in c_b*dF/dy
            ChState y,                 //< current state y
            double T,                   //< current time T
            double dt,                  //< timestep (if needed)
            bool force_state_scatter = true,  ///< if false, y and T are not scattered to the system
            bool force_setup = true           ///< if true, call the solver's Setup() function
        );

        /// Increment a vector R (usually the residual in a Newton Raphson iteration
        /// for solving an implicit integration step) with a term that has H multiplied a given vector w:
        ///    R += c*H*w
        void LoadResidual_Hv(ref ChVectorDynamic<double> R,        //< result: the R residual, R += c*M*v
                                 ChVectorDynamic<double> v,  //< the v vector
                                 double c               //< a scaling factor
                                 );

        /// Increment a vector R (usually the residual in a Newton Raphson iteration
        /// for solving an implicit integration step) with the term c*F:
        ///    R += c*F
        void LoadResidual_F(ref ChVectorDynamic<double> R,  //< result: the R residual, R += c*F
                                double c         //< a scaling factor
                                );

        /// Increment a vector R (usually the residual in a Newton Raphson iteration
        /// for solving an implicit integration step) with the term Cq'*L:
        ///    R += c*Cq'*L
        void LoadResidual_CqL(ref ChVectorDynamic<double> R,        //< result: the R residual, R += c*Cq'*L
                                  ChVectorDynamic<double> L,  //< the L vector
                                  double c               //< a scaling factor
                                  );

        /// Increment a vector Qc (usually the residual in a Newton Raphson iteration
        /// for solving an implicit integration step, constraint part) with the term C:
        ///    Qc += c*C
        void LoadConstraint_C(ref ChVectorDynamic<double> Qc,        //< result: the Qc residual, Qc += c*C
                                  double c,               //< a scaling factor
                                  bool do_clamp = false,  //< enable optional clamping of Qc
                                  double mclam = 1e30     //< clamping value
                                  );

        /// Increment a vector Qc (usually the residual in a Newton Raphson iteration
        /// for solving an implicit integration step, constraint part) with the term Ct = partial derivative dC/dt:
        ///    Qc += c*Ct
        void LoadConstraint_Ct(ref ChVectorDynamic<double> Qc,  //< result: the Qc residual, Qc += c*Ct
                                       double c          //< a scaling factor
                                       );

    }

    /// Special subcase: II-order differential system.
    /// Interface class for all objects that support time integration with state y that is second order:
    ///     y = {x, v} , dy/dt={v, a}
    /// with positions x, speeds v=dx/dt, and accelerations a=ddx/dtdt.
    /// Such systems permit the use of special integrators that can exploit the particular system structure.
    public interface ChIntegrableIIorder : ChIntegrable
    {

        /// Return the number of position coordinates x in y = {x, v}
        int GetNcoords_x();// { return 0; }

        /// Return the number of speed coordinates of v in y = {x, v} and  dy/dt={v, a}
        /// This is a base implementation that works in many cases where dim(v) = dim(x), but
        /// might be less ex. if x uses quaternions and v uses angular vel.
        int GetNcoords_v();// { return GetNcoords_x(); }

        /// Return the number of acceleration coordinates of a in dy/dt={v, a}
        /// This is a default implementation that works in almost all cases, as dim(a) = dim(v),
        int GetNcoords_a();// { return GetNcoords_v(); }

        /// Set up the system state with separate II order components x, v, a
        /// for y = {x, v} and  dy/dt={v, a}
        void StateSetup(ref ChState x, ref ChStateDelta v, ref ChStateDelta a);

        new void StateIncrement(ref ChState y_new,         //< resulting y_new = y + Dy
                        ChState y,       //< initial state y
                        ChStateDelta Dy  //< state increment Dy
                        );

        /// Increment state array:  x_new = x + dx    for x in    Y = {x, dx/dt}
        /// This is a base implementation that works in many cases, but it can be overridden
        /// in the case that x contains quaternions for rotations
        /// NOTE: the system is not updated automatically after the state increment, so one might
        /// need to call StateScatter() if needed.
        void StateIncrementX(ref ChState x_new,         //< resulting x_new = x + Dx
                                 ChState x,       //< initial state x
                                 ChStateDelta Dx  //< state increment Dx
                                 );

        /// Solve for accelerations: a = f(x,v,t)
        /// Given current state y={x,v} , computes acceleration a in the state derivative dy/dt={v,a} and
        /// lagrangian multipliers L (if any).
        /// NOTES
        ///  - some solvers (ex in DVI) cannot compute a classical derivative dy/dt when v is a function
        ///    of bounded variation, and f or L are distributions (e.g., when there are impulses and
        ///    discontinuities), so they compute a finite Dv through a finite dt. This is the reason why
        ///    this function has an optional parameter dt. In a DVI setting, one computes Dv, and returns
        ///    Dv*(1/dt) here in Dvdt parameter; if the original Dv has to be known, just multiply Dvdt*dt later.
        ///    The same for impulses: a DVI would compute impulses I, and return L=I*(1/dt).
        ///  - derived classes must take care of calling StateScatter(y,T) before computing Dy, only if
        ///    force_state_scatter = true (otherwise it is assumed state is already in sync)
        ///  - derived classes must take care of resizing Dv if needed.
        ///
        /// This function must return true if successful and false otherwise.
        ///
        /// This default implementation uses the same functions already used for implicit integration.
        /// WARNING: this implementation avoids the computation of the analytical expression for Qc, but
        /// at the cost of three StateScatter updates.
        bool StateSolveA(ref ChStateDelta Dvdt,              ///< result: computed a for a=dv/dt
                             ref ChVectorDynamic<double> L,            ///< result: computed lagrangian multipliers, if any
                             ChState x,                ///< current state, x
                             ChStateDelta v,           ///< current state, v
                             double T,                  ///< current time T
                             double dt,                 ///< timestep (if needed)
                             bool force_state_scatter = true  ///< if false, x,v and T are not scattered to the system
                             );

        /// Scatter the states from the provided arrays to the system.
        /// This function is called by time integrators all times they modify the Y state.
        /// In some cases, the ChIntegrable object might contain dependent data structures
        /// that might need an update at each change of Y. If so, this function must be overridden.
        void StateScatter(ChState x, ChStateDelta v, double T);

        /// From system to state y={x,v}
        /// Optionally, they will copy system private state, if any, to y={x,v}
        void StateGather(ref ChState x, ref ChStateDelta v, ref double T);

        /// Scatter the acceleration from the provided array to the system.
        /// Optional: the integrable object might contain last computed state derivative, some integrators might use it.
        void StateScatterAcceleration(ChStateDelta a);

        //
        // Functions required by implicit integration schemes
        //

        /// Assuming an explicit ODE in the form
        ///        M*a = F(x,v,t)
        /// Assuming an explicit DAE in the form
        ///        M*a = F(x,v,t) + Cq'*L
        ///     C(x,t) = 0
        /// this must compute the solution of the change Du (in a or v or x) for a Newton
        /// iteration within an implicit integration scheme.
        /// If in ODE case:
        ///  Du = [ c_a*M + c_v*dF/dv + c_x*dF/dx ]^-1 * R
        ///  Du = [ G ]^-1 * R
        /// If with DAE constraints:
        ///  |Du| = [ G   Cq' ]^-1 * | R |
        ///  |DL|   [ Cq  0   ]      | Qc|
        /// where R is a given residual, dF/dv and dF/dx, dF/dv are jacobians (that are also
        /// -R and -K, damping and stiffness (tangent) matrices in many mechanical problems, note the minus sign!).
        /// It is up to the derived class how to solve such linear system.
        ///
        /// This function must return true if successful and false otherwise.
        bool StateSolveCorrection(
            ref ChStateDelta Dv,                 //< result: computed Dv
            ref ChVectorDynamic<double> L,             //< result: computed lagrangian multipliers, if any
            ChVectorDynamic<double> R,        //< the R residual
            ChVectorDynamic<double> Qc,      //< the Qc residual
            double c_a,                 //< the factor in c_a*M
            double c_v,                 //< the factor in c_v*dF/dv
            double c_x,                 //< the factor in c_x*dF/dv
            ChState x,                 //< current state, x part
            ChStateDelta v,            //< current state, v part
            double T,                   //< current time T
            bool force_state_scatter = true,  //< if false, x,v and T are not scattered to the system
            bool force_setup = true           //< if true, call the solver's Setup() function
        );

        /// Assuming   M*a = F(x,v,t) + Cq'*L
        ///         C(x,t) = 0
        /// increment a vector R (usually the residual in a Newton Raphson iteration
        /// for solving an implicit integration step) with a term that has M multiplied a given vector w:
        ///    R += c*M*w
        void LoadResidual_Mv(ref ChVectorDynamic<double> R,        //< result: the R residual, R += c*M*v
                                 ChVectorDynamic<double> w,  //< the w vector
                                 double c               //< a scaling factor
                                 );   

        /// Solve for state derivatives: dy/dt = f(y,t).
        /// (overrides base - just a fallback to enable using with plain 1st order timesteppers)
        /// PERFORMANCE WARNING! temporary vectors allocated on heap. This is only to support
        /// compatibility with 1st order integrators.
        new bool StateSolve(ref ChStateDelta dydt,              //< result: computed dydt
                            ref ChVectorDynamic<double> L,            //< result: computed lagrangian multipliers, if any
                            ChState y,                //< current state y
                            double T,                  //< current time T
                            double dt,                 //< timestep (if needed, ex. in DVI)
                            bool force_state_scatter = true  //< if false, y and T are not scattered to the system
                            ); 

    }
}
