import numpy as np
from scipy.optimize import minimize_scalar

from fit_wheel_speed_bag import fit_resistance, resistance, coast_response, quiet_carrier


def test_recovers_directional_resistance_without_an_unidentifiable_extra_bias():
    speeds=np.array([-30.,-20.,-10.,-5.,-2.,2.,5.,10.,20.,30.])
    coefficients=[.08,.10,.004]
    torque=resistance(speeds,coefficients)
    fit=fit_resistance(speeds,torque)
    assert np.allclose(fit['coefficients'],coefficients,atol=1e-10)
    assert fit_resistance([1.,2.,3.],[.1,.2,.3]) is None


def test_coast_fit_recovers_inertia_and_stops_without_reversing():
    t=np.linspace(0.,1.,1001)
    coefficients=[.08,.10,.004]
    observed=coast_response(t,-10.,.003,coefficients)
    result=minimize_scalar(lambda log_j:np.mean((coast_response(t,-10.,np.exp(log_j),coefficients)-observed)**2),
                          bounds=(np.log(1e-6),np.log(.1)),method='bounded')
    assert np.isclose(np.exp(result.x),.003,rtol=1e-4)
    assert np.all(observed<=0.)
    assert observed[-1]==0.
    assert np.allclose(coast_response(np.array([0.,.1,.2]),1.,.01,[.1,.1,0.]),[1.,0.,0.])


def test_passive_leg_motion_is_not_accepted_as_a_fixed_carrier():
    q=np.zeros((1000,4));v=np.zeros_like(q)
    assert quiet_carrier(q,v)
    q[:,0]=np.linspace(0,.04,1000)
    assert not quiet_carrier(q,v)
    q[:]=0;v[:,1]=.2
    assert not quiet_carrier(q,v)
