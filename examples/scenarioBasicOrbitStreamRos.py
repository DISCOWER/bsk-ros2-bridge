#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

r"""
Overview
--------

This script duplicates the basic orbit simulation in the scenario :ref:`scenarioBasicOrbit`.
The difference is that this version allows for the Basilisk simulation data to be live streamed to the
:ref:`vizard` visualization program, with optional 2-way communication with Vizard (live user inputs to
the simulation).

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioBasicOrbitStreamRos.py

To enable live data streaming and/or broadcast streaming, the ``enableUnityVisualization()`` method is provided
with ``liveStream`` and ``broadcastStream`` argument using::

    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        , liveStream=True
                                        , broadcastStream=True)

When starting Basilisk simulation it prints now to the terminal that it is trying to connect to Vizard::

    Waiting for Vizard at tcp://localhost:5556

Copy ``tcp://localhost:5556`` and open the Vizard application.  Enter this address in the connection field and select
"Direct Communication" mode as well as "Live Streaming".  After this the Basilisk simulation resumes and
will live stream the data to Vizard.

.. figure:: /_images/static/vizard-ImgStream.jpg
   :align: center
   :scale: 50 %

   Vizard Direct Communication Panel Illustration


To avoid the simulation running too quickly, this tutorial example script includes the ``clock_sync`` module that
enables a 50x realtime mode using::

    clockSync = clock_synch.ClockSynch()
    clockSync.accelFactor = 50.0
    scSim.AddModelToTask(simTaskName, clockSync)

This way a 10s simulation time step will take 0.2 seconds with the 50x speed up factor.

"""


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft() and gravity modules.  Illustrates
#           a 3-DOV spacecraft on a range of orbit types with live Vizard data streaming
#           and 2-way communication with Vizard.
# Author:   Hanspeter Schaub
# Creation Date:  Sept. 29, 2019
#

import os

import matplotlib.pyplot as plt
import numpy as np
import datetime as dt
import time

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

# import simulation related support
from Basilisk.simulation import spacecraft
# general support file with common unit test functions
# import general simulation support files
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                simIncludeGravBody, unitTestSupport, vizSupport, simIncludeThruster)
from Basilisk.simulation import simSynch
from Basilisk.architecture import messaging
from Basilisk.simulation import thrusterDynamicEffector, extForceTorque
try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass

# Import the ROS bridge handler
# from Basilisk.utilities import rosBridgeHandler
import sys, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
# Add the parent directory to the path to access bsk_module
sys.path.append(os.path.join(path, '..'))
from bsk_module.rosBridgeHandler import RosBridgeHandler

def run(show_plots, liveStream, broadcastStream, timeStep, orbitCase, useSphericalHarmonics, planetCase, accelFactor=50.0):
    """
    Args:
        show_plots (bool): Determines if the script should display plots
        liveStream (bool): Determines if the script should use live data streaming
        broadcastStream (bool): Determines if the script should broadcast messages for listener Vizards to pick up.
        timeStep (double): Integration update time in seconds
        orbitCase (str):

            ======  ============================
            String  Definition
            ======  ============================
            'LEO'   Low Earth Orbit
            'GEO'   Geosynchronous Orbit
            'GTO'   Geostationary Transfer Orbit
            ======  ============================

        useSphericalHarmonics (Bool): False to use first order gravity approximation: :math:`\\frac{GMm}{r^2}`

        planetCase (str): {'Earth', 'Mars'}
        accelFactor (float): Real-time acceleration factor for simulation clock (1.0 = real time, 50.0 = 50x faster)
    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(timeStep)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bskSat"
    # Set spacecraft properties
    scObject.hub.mHub = 17.8  # kg, spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m, center of mass offset
    scObject.hub.IHubPntBc_B = [[0.314, 0.0, 0.0],
                                [0.0, 0.314, 0.0],
                                [0.0, 0.0, 0.314]]  # kg*m^2, spacecraft inertia

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # --- Add ROS Bridge Handler ---
    # Create ROS bridge handler - it will automatically discover BSK message types
    rosHandlerModule = RosBridgeHandler(namespace="bskSat")
    rosHandlerModule.ModelTag = "ros_bridge_handler"

    # Add message readers/writers only if the types were discovered
    rosHandlerModule.add_bsk_msg_reader('SCStatesMsgPayload', 'scStateInMsg', 'sc_states')
    rosHandlerModule.add_bsk_msg_reader('CmdForceBodyMsgPayload', 'cmdForceBodyInMsg', 'cmd_force_body')
    rosHandlerModule.add_bsk_msg_reader('CmdTorqueBodyMsgPayload', 'cmdTorqueBodyInMsg', 'cmd_torque_body')
    
    # Add writers for command topics (receiving commands from ROS2)
    rosHandlerModule.add_bsk_msg_writer('CmdForceBodyMsgPayload', 'cmdForceBodyOutMsg', 'cmd_force_body')
    rosHandlerModule.add_bsk_msg_writer('CmdTorqueBodyMsgPayload', 'cmdTorqueBodyOutMsg', 'cmd_torque_body')

    # Connect spacecraft state output to bridge handler input
    rosHandlerModule.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    rosHandlerModule.cmdForceBodyInMsg.subscribeTo(rosHandlerModule.cmdForceBodyOutMsg)
    rosHandlerModule.cmdTorqueBodyInMsg.subscribeTo(rosHandlerModule.cmdTorqueBodyOutMsg)

    # Create external force and torque rosHandlerModule to apply ROS commands
    extForceTorqueModule = extForceTorque.ExtForceTorque()
    extForceTorqueModule.ModelTag = "externalDisturbance"
    extForceTorqueModule.cmdForceBodyInMsg.subscribeTo(rosHandlerModule.cmdForceBodyOutMsg)
    extForceTorqueModule.cmdTorqueInMsg.subscribeTo(rosHandlerModule.cmdTorqueBodyOutMsg)
    scObject.addDynamicEffector(extForceTorqueModule)

    # Add to simulation task
    scSim.AddModelToTask(simTaskName, rosHandlerModule)
    scSim.AddModelToTask(simTaskName, extForceTorqueModule)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    if planetCase == 'Mars':
        planet = gravFactory.createMarsBarycenter()
        planet.isCentralBody = True           # ensure this is the central gravitational body
        if useSphericalHarmonics:
            planet.useSphericalHarmonicsGravityModel(bskPath + '/supportData/LocalGravData/GGM2BData.txt', 100)

    else:  # Earth
        planet = gravFactory.createEarth()
        planet.isCentralBody = True          # ensure this is the central gravitational body
        if useSphericalHarmonics:
            planet.useSphericalHarmonicsGravityModel(bskPath + '/supportData/LocalGravData/GGM03S-J2-only.txt', 2)
    mu = planet.mu

    # attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    rGEO = 42000. * 1000     # meters
    if orbitCase == 'GEO':
        oe.a = rGEO
        oe.e = 0.00001
        oe.i = 0.0 * macros.D2R
    elif orbitCase == 'GTO':
        oe.a = (rLEO + rGEO) / 2.0
        oe.e = 1.0 - rLEO / oe.a
        oe.i = 0.0 * macros.D2R
    else:                   # LEO case, default case 0
        oe.a = 2.5*rLEO
        oe.e = 0.10
        oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # this stores consistent initial orbit elements
    # with circular or equatorial orbit, some angles are arbitrary

    #
    #   initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N

    # # Configure thruster
    # thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    # scSim.AddModelToTask(simTaskName, thrusterSet)

    # # Make a fresh thruster factory instance, this is critical to run multiple times
    # thFactory = simIncludeThruster.thrusterFactory()
    # thFactory.create('MOOG_Monarc_22_6', [0, 0, 0], [0, -1.5, 0])
    # thrModelTag = "ACSThrusterDynamics"
    # thFactory.addToSpacecraft(thrModelTag, thrusterSet, scObject)

    # thrMsgData = messaging.THRArrayOnTimeCmdMsgPayload()
    # thrMsgData.OnTimeRequest = [0, 0, 0]
    # thrMsg = messaging.THRArrayOnTimeCmdMsg()
    # thrMsg.write(thrMsgData)
    # thrusterSet.cmdsInMsg.subscribeTo(thrMsg)

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    if useSphericalHarmonics:
        simulationTime = macros.sec2nano(3. * P)
    else:
        simulationTime = macros.sec2nano(1 * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    if useSphericalHarmonics:
        numDataPoints = 400
    else:
        numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    dataLog = scObject.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataLog)

    if vizSupport.vizFound:
        # create spacecraft data container
        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = scObject.ModelTag
        scData.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

        clockSync = simSynch.ClockSynch()
        clockSync.accelFactor = accelFactor
        scSim.AddModelToTask(simTaskName, clockSync)

        # Configure Vizard, using liveStream and broadcastStream options
        viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                                #   , thrEffectorList=thrusterSet
                                                  , thrColors=vizSupport.toRGBA255("white")
                                                  , liveStream=liveStream
                                                  , broadcastStream=broadcastStream
                                                  )
        # Set key listeners
        viz.settings.keyboardLiveInput = "bxpz"

        # To set 2-way port:
        viz.reqComProtocol = "tcp"
        viz.reqComAddress = "localhost"
        viz.reqPortNumber = "5556"

        # To set broadcast port:
        viz.pubComProtocol = "tcp"
        viz.pubComAddress = "localhost"
        viz.pubPortNumber = "5570"

        # Del viz.vizEventDialogs[:] at the start of the sim
        viz.vizEventDialogs.clear()

        # "Subscriber" Vizards will pick up the main settings at this frequency
        viz.broadcastSettingsSendDelay = 2  # seconds

    #
    #   initialize Simulation:  This function clears the simulation log, and runs the self_init()
    #   cross_init() and reset() routines on each module.
    #   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
    #   then the all messages are auto-discovered that are shared across different BSK threads.
    #
    scSim.InitializeSimulation()

    # This is the execution loop. BSK executes at a frequency governed by [n * simulationTimeStep].
    incrementalStopTime = 0

    # Scenario specific flag
    continueBurn = False

    priorKeyPressTime = dt.datetime.now()
    while incrementalStopTime < simulationTime:
        # Here, I only want to run a single BSK timestep before checking for user responses.
        # If the 'end' flag is set, exit the scenario.
        # If the 'pause' flag is set, only update vizInterface module and reset clockSync.
        if vizSupport.endFlag:
            exit(0)
        elif vizSupport.pauseFlag:
            viz.UpdateState(incrementalStopTime)
            clockSync.Reset(0)
        else:
            incrementalStopTime += simulationTimeStep
            scSim.ConfigureStopTime(incrementalStopTime)
            scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(3):
        plt.plot(dataLog.times() * macros.NANO2SEC / P, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1" + orbitCase + str(int(useSphericalHarmonics))+ planetCase
    figureList[pltName] = plt.figure(1)

    if useSphericalHarmonics is False:
        # draw orbit in perifocal frame
        b = oe.a * np.sqrt(1 - oe.e * oe.e)
        p = oe.a * (1 - oe.e * oe.e)
        plt.figure(2, figsize=tuple(np.array((1.0, b / oe.a)) * 4.75), dpi=100)
        plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
        # draw the planet
        fig = plt.gcf()
        ax = fig.gca()
        if planetCase == 'Mars':
            planetColor = '#884400'
        else:
            planetColor = '#008800'
        planetRadius = planet.radEquator / 1000
        ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
        # draw the actual orbit
        rData = []
        fData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
            rData.append(oeData.rmag)
            fData.append(oeData.f + oeData.omega - oe.omega)
        plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=3.0
                 )
        # draw the full osculating orbit from the initial conditions
        fData = np.linspace(0, 2 * np.pi, 100)
        rData = []
        for idx in range(0, len(fData)):
            rData.append(p / (1 + oe.e * np.cos(fData[idx])))
        plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555'
                 )
        plt.xlabel('$i_e$ Cord. [km]')
        plt.ylabel('$i_p$ Cord. [km]')
        plt.grid()

    else:
        plt.figure(2)
        fig = plt.gcf()
        ax = fig.gca()
        ax.ticklabel_format(useOffset=False, style='plain')
        smaData = []
        for idx in range(0, len(posData)):
            oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
            smaData.append(oeData.a / 1000.)
        plt.plot(posData[:, 0] * macros.NANO2SEC / P, smaData, color='#aa0000',
                 )
        plt.xlabel('Time [orbits]')
        plt.ylabel('SMA [km]')

    pltName = fileName + "2" + orbitCase + str(int(useSphericalHarmonics)) + planetCase
    figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        False,       # show_plots
        True,       # liveStream
        True,        # broadcastStream
        0.01,      # time step (s)
        'LEO',       # orbit Case (LEO, GTO, GEO)
        False,       # useSphericalHarmonics
        'Earth',     # planetCase (Earth, Mars)
        1.0          # accelFactor (default 50x, set to 1.0 for real time)
    )
