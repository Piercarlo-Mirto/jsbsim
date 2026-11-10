# TestTurbopropTrim.py
#
# Regression tests for turboprop engine power during JSBSim's trim phase.
#
# Copyright (c) 2026 Piercarlo Mirto
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>
#

from JSBSim_utils import JSBSimTestCase, RunTest


class TestTurbopropTrim(JSBSimTestCase):
    # Trim suspends integration, which makes FGFDMExec pass TotalDeltaT == 0
    # to every engine. FGTurboProp::Calculate() has always entered a dedicated
    # tpTrim phase in that case, but until now there was no handler for it in
    # its phase switch, so it fell through to "default: HP = 0" and delivered
    # zero power regardless of throttle. That made the throttle trim axis
    # untrimmable on any turboprop (udot never changes sign with throttle).
    #
    # These tests check that turboprop power now responds to throttle whenever
    # integration is suspended, using fdm.run_ic() (which suspends integration
    # for a single frame, exactly like FGTrim does for each axis evaluation)
    # rather than a full simulation/do_simple_trim run. A full trim repeatedly
    # drives the throttle axis through FGPropulsion::GetSteadyState(), which
    # exposes a separate, pre-existing instability in FGPropeller's
    # constant-speed governor under large time steps once real power actually
    # flows through it (previously unreachable, since turboprop trim always
    # failed at zero thrust beforehand) -- that is a distinct defect, tracked
    # separately, and is intentionally not exercised here.
    def test_power_responds_to_throttle_when_integration_suspended(self):
        fdm = self.create_fdm()
        fdm.load_model('pc7')
        fdm['ic/vc-kts'] = 120.0
        fdm['ic/h-sl-ft'] = 3000.0
        fdm['ic/gamma-deg'] = 0.0

        powers = []
        for throttle in (0.0, 0.5, 1.0):
            fdm['fcs/throttle-cmd-norm'] = throttle
            fdm.run_ic()
            powers.append(fdm['propulsion/engine/power-hp'])

        # Idle should deliver negligible power, and power must increase
        # monotonically with throttle rather than being pinned at zero.
        self.assertAlmostEqual(powers[0], 0.0, delta=1.0)
        self.assertLess(powers[0], powers[1])
        self.assertLess(powers[1], powers[2])


RunTest(TestTurbopropTrim)
