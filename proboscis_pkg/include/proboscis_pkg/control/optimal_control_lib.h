#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include "proboscis_pkg/control/B_tau_sol1.h"

#define N_STRAIN_MODES  6
#define N_ACT           7

// Rest Configuration
#define KX0 0.0
#define KY0 0.0
#define KZ0 0.0
#define SX0 0.0
#define SY0 0.0
#define SZ0 1.0 

// Global Variable
const Eigen::MatrixXd W_xi = Eigen::MatrixXd::Identity(N_STRAIN_MODES, N_STRAIN_MODES);
const Eigen::MatrixXd W_tau = Eigen::MatrixXd::Identity(N_ACT, N_ACT);
const Eigen::MatrixXd stiff_matrix = Eigen::MatrixXd::Identity(N_STRAIN_MODES, N_STRAIN_MODES);
Eigen::VectorXd xi_des(N_STRAIN_MODES);
Eigen::VectorXd xi_rest(N_STRAIN_MODES);

namespace ifopt 
{
  using namespace Eigen;

  class ExVariables : public VariableSet 
  {
    private:
      double x0_, x1_, x2_, x3_, x4_, x5_, x6_, x7_, x8_, x9_, x10_, x11_, x12_;
    
    public:
    // Every variable set has a name, here "var_set1". this allows the constraints
    // and costs to define values and Jacobians specifically w.r.t this variable set.
    ExVariables() : ExVariables("var_set1") {};
    ExVariables(const std::string& name) : VariableSet(N_STRAIN_MODES + N_ACT, name)
    {
      // the initial values where the NLP starts iterating from
      // --- Strain Modes --- //
      x0_ = KX0;
      x1_ = KY0;
      x2_ = KZ0;
      x3_ = SX0;
      x4_ = SY0;
      x5_ = SZ0;
      // --- Actuation --- //
      x6_ = 0.0;
      x7_ = 0.0;
      x8_ = 0.0;
      x9_ = 0.0;
      x10_ = 0.0;
      x11_ = 0.0;
      x12_ = 0.0;
    }

    // Here is where you can transform the Eigen::Vector into whatever
    // internal representation of your variables you have (here two doubles, but
    // can also be complex classes such as splines, etc..
    void SetVariables(const VectorXd& x) override
    {
      // --- Strain Modes --- //
      x0_ = x(0);
      x1_ = x(1);
      x2_ = x(2);
      x3_ = x(3);
      x4_ = x(4);
      x5_ = x(5);
      // --- Actuation --- //
      x6_ = x(6);
      x7_ = x(7);
      x8_ = x(8);
      x9_ = x(9);
      x10_ = x(10);
      x11_ = x(11);
      x12_ = x(12);
    };

    // Here is the reverse transformation from the internal representation to
    // to the Eigen::Vector
    VectorXd GetValues() const override
    {
      VectorXd state(N_STRAIN_MODES + N_ACT);
      state <<    x0_, x1_, x2_, x3_, x4_, x5_, 
                  x6_, x7_, x8_, x9_, x10_, x11_, x12_; 
      return state;
    };

    // Each variable has an upper and lower bound set here
    VecBound GetBounds() const override
    {
      VecBound bounds(GetRows());
      for(int i = 0; i < N_STRAIN_MODES + N_ACT; i++)
      {
          // No Bound on Strain Modes
          if(i < N_STRAIN_MODES)
          {
              bounds.at(i) = NoBound;
          }
          // Only negative tau (cables)
          else
          {
              bounds.at(i) = Bounds(-inf, 0.0);
          }  
      }
      return bounds;
    }
  };


  class ExConstraint : public ConstraintSet
  {
    public:
      ExConstraint() : ExConstraint("constraint1") {}

      // This constraint set just contains 1 constraint, however generally
      // each set can contain multiple related constraints.
      ExConstraint(const std::string& name) : ConstraintSet(N_STRAIN_MODES, name) {}

      // The constraint value minus the constant value "1", moved to bounds.
      VectorXd GetValues() const override
      {
        VectorXd g(GetRows());
        VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
        
        // Compute Constraint
        VectorXd xi = x.head(N_STRAIN_MODES);
        VectorXd tau = x.tail(N_ACT);

        VectorXd f_xi = stiff_matrix.inverse()*get_ActMat(xi, 0.7)*tau + xi_rest;
        g = f_xi - xi;
        return g;
      };

      // The only constraint in this set is an equality constraint to 1.
      // Constant values should always be put into GetBounds(), not GetValues().
      // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
      VecBound GetBounds() const override
      {
        // di sicuro da errore
        VecBound b(GetRows());
        for(int i = 0; i < N_STRAIN_MODES; i++)
        {
          b.at(i) = Bounds(0.0, 0.0); // equality constraint to == 0
        }    
        return b;
      }

      // This function provides the first derivative of the constraints.
      // In case this is too difficult to write, you can also tell the solvers to
      // approximate the derivatives by finite differences and not overwrite this
      // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
      // Attention: see the parent class function for important information on sparsity pattern.
      void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
      {
        // This function is empty because is not to override.
        // Jacobian is computed by approximation (finite differences)
        return;
      } /*TO NOT OVERRIDE*/
  };


  class ExCost: public CostTerm 
  {
    public:
      ExCost() : ExCost("cost_term1") {}
      ExCost(const std::string& name) : CostTerm(name) {}

      double GetCost() const override
      {
        VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
        VectorXd xi = x.head(N_STRAIN_MODES);
        VectorXd tau = x.tail(N_ACT);
        VectorXd fun_cost = 0.5*((xi - xi_des).transpose()*W_xi*(xi - xi_des)) + 0.5*(tau.transpose()*W_tau*tau);
        return fun_cost(0);
      };

      void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
      {
        if (var_set == "var_set1") {
          VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();

            for(int i = 0; i < N_STRAIN_MODES + N_ACT; i++)
            {
                if(i < N_STRAIN_MODES)
                    jac.coeffRef(0, i) = W_xi(i, i)*x(i);
                else
                    jac.coeffRef(0, i) = W_tau(i, i)*x(i);
            }
        }
      }
  };

} // namespace opt