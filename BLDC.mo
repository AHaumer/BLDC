within ;
package BLDC
  package UsersGuide "User's Guide"
    extends Modelica.Icons.Information;

    class ReleaseNotes "Release Notes"
      extends Modelica.Icons.ReleaseNotes;
      annotation (Documentation(info="<html>

<h4>v1.0.0 2021-12-10</h4>
<ul>
<li>First fully working version</li>
<li>Under investigation: current measurement for current control</li>
</ul>

</html>"));
    end ReleaseNotes;

    class Contact "Contact"
      extends Modelica.Icons.Contact;
      annotation (Documentation(info="<html>
<p>
Prof. Anton Haumer<br>
email: <a HREF=\\\"mailto:anton.haumer@oth-regensburg.de\\\">anton.haumer@oth-regensburg.de</a><br>
<a href=\"https://www.oth-regensburg.de/\">OTH Regensburg</a> (Technical University of Applied Sciences), Faculty of Electrical Engineering and Information Technology
</p>
</html>"));
    end Contact;
  annotation (DocumentationClass=true, Documentation(info="<html>
<p>
This library is developed at <a href=\"https://www.oth-regensburg.de/\">OTH Regensburg</a> (Technical University of Applied Sciences), 
Faculty of Electrical Engineering and Information Technology, Prof. Anton Haumer.
</p>
<p>
The goal is to implement models for brushless DC machines (BLDC), 
ie. permanent magnet synchronous machines under the restriction of spatial sinusoidal magnetic field distribution 
and block commuation derived from teh signals of a Hall sensor. 
</p>
<h4>Note:</h4>
<p>
Due to the electronic commutator, i.e. determining the active sector, the models are restricted to <code>m=3</code> phases.
</p>
</html>"));
  end UsersGuide;

  package Examples "Demo examples"
    extends Modelica.Icons.ExamplesPackage;
    model DemoEncoder "Demonstrate various encoder / resolver models"
      extends Modelica.Icons.Example;
      import Modelica.Constants.pi;
      parameter Integer p(final min=1)=2 "Number of pole pairs";
      parameter Integer pRev(final min=1)=128 "Pulses per revolution";
      parameter Modelica.Units.SI.Angle phi0=0 "Initial mechanical angle (zero position)";
      Modelica.Blocks.Sources.Trapezoid refFrequency(
        amplitude=100,
        rising=1,
        width=0.5,
        falling=1,
        period=3,
        nperiod=2,
        offset=-50,
        startTime=-0.5)
        annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
      Modelica.Blocks.Continuous.Integrator f2pos(k=2*pi/p, y_start=phi0)
        annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
      Modelica.Mechanics.Rotational.Sources.Position position(exact=true)
        annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor
        annotation (Placement(transformation(extent={{10,60},{30,80}})));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
        annotation (Placement(transformation(extent={{10,40},{30,60}})));
      Sensors.HallSensor hallSensor(p=p, phi0=phi0)
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=180,
            origin={20,20})));
      CommonBlocks.UnwrapAngle unwrapAngle(phi0=phi0)
        annotation (Placement(transformation(extent={{50,20},{70,40}})));
      Sensors.SinCosResolver sinCosResolver(p=p, phi0=phi0)
        annotation (Placement(transformation(extent={{30,-20},{10,0}})));
      Utilities.SinCosEvaluation sinCosEvaluation(phi0=phi0)
        annotation (Placement(transformation(extent={{50,-20},{70,0}})));
      Sensors.IncrementalEncoder incrementalEncoder(pRev=pRev)
        annotation (Placement(transformation(extent={{30,-50},{10,-30}})));
      Utilities.EncoderEvaluation encoderEvaluation(pRev=pRev, phi0=phi0)
        annotation (Placement(transformation(extent={{50,-50},{70,-30}})));
      Utilities.EncoderPulseCount encoderPulseCount(pRev=pRev, phi0=phi0)
        annotation (Placement(transformation(extent={{50,-80},{70,-60}})));
    equation
      connect(position.flange, sinCosResolver.flange)
        annotation (Line(points={{-10,0},{0,0},{0,-10},{10,-10}},
                                                                color={0,0,0}));
      connect(f2pos.y, position.phi_ref)
        annotation (Line(points={{-39,0},{-32,0}}, color={0,0,127}));
      connect(position.flange, incrementalEncoder.flange)
        annotation (Line(points={{-10,0},{0,0},{0,-40},{10,-40}}, color={0,0,0}));
      connect(incrementalEncoder.y, encoderEvaluation.u)
        annotation (Line(points={{31,-40},{48,-40}}, color={255,0,255}));
      connect(sinCosResolver.y, sinCosEvaluation.u)
        annotation (Line(points={{31,-10},{48,-10}},
                                                   color={0,0,127}));
      connect(position.flange, hallSensor.flange)
        annotation (Line(points={{-10,0},{0,0},{0,20},{10,20}},color={0,0,0}));
      connect(incrementalEncoder.y, encoderPulseCount.u) annotation (Line(points={{31,-40},
              {40,-40},{40,-70},{48,-70}},      color={255,0,255}));
      connect(refFrequency.y, f2pos.u)
        annotation (Line(points={{-69,0},{-62,0}}, color={0,0,127}));
      connect(hallSensor.y, unwrapAngle.u)
        annotation (Line(points={{31,26},{40,26},{40,30},{48,30}},
                                                   color={0,0,127}));
      connect(position.flange, angleSensor.flange)
        annotation (Line(points={{-10,0},{0,0},{0,70},{10,70}}, color={0,0,0}));
      connect(position.flange, speedSensor.flange)
        annotation (Line(points={{-10,0},{0,0},{0,50},{10,50}}, color={0,0,0}));
      annotation (experiment(
          StopTime=3,
          Interval=0.0001,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"), Documentation(info="<html>
<p>
The reference frequency signal is integrated to obtain the angle, which is measured by:
</p>
<ul>
<li>an ideal <a href=\"modelica://Modelica.Mechanics.Rotational.Sensors.AngleSensor\">AngleSensor</a> and 
    an ideal <a href=\"modelica://Modelica.Mechanics.Rotational.Sensors.SpeedSensor\">SpeedSensor</a>, </li>
<li>a <a href=\"modelica://BLDC.Sensors.HallSensor\">HallSensor</a>,</li>
<li>a <a href=\"modelica://BLDC.Sensors.SinCosResolver\">SinCosResolver</a>, 
    the outputs are interpreted by the block <a href=\"modelica://BLDC.Utilities.SinCosEvaluation\">SinCosEvaluation</a>,</li>
<li>an <a href=\"modelica://BLDC.Sensors.IncrementalEncoder\">IncrementalEncoder</a>, 
    the outputs are interpreted by the blocks <a href=\"modelica://BLDC.Utilities.EncoderEvaluation\">EncoderEvaluation</a> 
    and <a href=\"modelica://BLDC.Utilities.EncoderPulseCount\">EncoderPulseCount</a>.</li>
</ul>
<p>
For <code>p=1</code>, all results are identical. 
For <code>p&gt;1</code>, the results of the HallSensor and the SinCosEvaluation are p times the other results. 
</p>
</html>"));
    end DemoEncoder;

    model DemoBLDCVoltages "Test example: Demonstrate BLDC voltages"
      extends Modelica.Icons.Example;
      import Modelica.Units.SI;
      import Modelica.Constants.pi;
      constant Integer m=3 "Number of phases";
      parameter Integer p(final min=1)=2 "Number of pole pairs";
      parameter SI.Voltage VDC=100 "Nominal DC voltage";
      parameter SI.Frequency fNominal=50 "Nominal frequqncy";
      parameter SI.AngularVelocity wNominal(displayUnit="rpm")=2*pi*fNominal/p "Nominal speed";
      Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(
        w_fixed=0.1*wNominal,  phi(start=0, fixed=true))
        annotation (Placement(transformation(extent={{-90,10},{-70,30}})));
      BLDC.Sensors.HallSensor hallSensor(p=p, m=m) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-50,20})));
      BLDC.Utilities.ElectronicCommutator electronicCommutator
        annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
      Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter(m=m)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,40})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
        annotation (Placement(transformation(extent={{10,70},{-10,90}})));
      Modelica.Electrical.Analog.Basic.Ground groundDC annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-20,70})));
      Modelica.Electrical.Polyphase.Sensors.VoltageSensor voltageSensor(m=m)
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={0,10})));
      Modelica.Electrical.Polyphase.Basic.Star star(m=m) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-20})));
      Modelica.Electrical.Machines.SpacePhasors.Blocks.ToSpacePhasor toSpacePhasor
        annotation (Placement(transformation(extent={{20,0},{40,20}})));
      Modelica.Electrical.Machines.SpacePhasors.Blocks.Rotator rotator
        annotation (Placement(transformation(extent={{50,0},{70,20}})));
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=1e6) annotation (
          Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-50})));
      Modelica.Electrical.Analog.Basic.Ground groundAC
        annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));
      Modelica.Blocks.Sources.BooleanConstant booleanConstant
        annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
    equation
      connect(constantVoltage.p, inverter.dc_p) annotation (Line(points={{10,80},
              {10,70},{6,70},{6,50}},color={0,0,255}));
      connect(electronicCommutator.fire_p, inverter.fire_p)
        annotation (Line(points={{-19,46},{-12,46}},
                                                 color={255,0,255}));
      connect(electronicCommutator.fire_n, inverter.fire_n)
        annotation (Line(points={{-19,34},{-12,34}},
                                                 color={255,0,255}));
      connect(hallSensor.yC, electronicCommutator.uC)
        annotation (Line(points={{-39,20},{-30,20},{-30,28}},
                                                     color={255,0,255}));
      connect(constantSpeed.flange, hallSensor.flange)
        annotation (Line(points={{-70,20},{-60,20}}, color={0,0,0}));
      connect(inverter.ac, voltageSensor.plug_p)
        annotation (Line(points={{-1.77636e-15,30},{-1.77636e-15,26},{0,26},{0,
              20}},                                color={0,0,255}));
      connect(voltageSensor.plug_n, star.plug_p)
        annotation (Line(points={{0,0},{0,-10}},     color={0,0,255}));
      connect(voltageSensor.v, toSpacePhasor.u)
        annotation (Line(points={{11,10},{18,10}},
                                                 color={0,0,127}));
      connect(toSpacePhasor.y, rotator.u)
        annotation (Line(points={{41,10},{48,10}},
                                                 color={0,0,127}));
      connect(hallSensor.y, rotator.angle) annotation (Line(points={{-39,14},{
              -30,14},{-30,-90},{60,-90},{60,-2}},
                                            color={0,0,127}));
      connect(constantVoltage.n, groundDC.p)
        annotation (Line(points={{-10,80},{-10,70}}, color={0,0,255}));
      connect(groundDC.p, inverter.dc_n)
        annotation (Line(points={{-10,70},{-6,70},{-6,50}}, color={0,0,255}));
      connect(star.pin_n, resistor.p)
        annotation (Line(points={{0,-30},{0,-40}},   color={0,0,255}));
      connect(resistor.n, groundAC.p)
        annotation (Line(points={{0,-60},{0,-70}}, color={0,0,255}));
      connect(booleanConstant.y, electronicCommutator.pwm) annotation (Line(points=
             {{-59,50},{-50,50},{-50,46},{-42,46}}, color={255,0,255}));
      annotation (experiment(
          Interval=1e-05,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"),                                      Documentation(
            info="<html>
<p>
Demonstrates how phase voltages are built from a constant DC voltage and Hall sensor signals, spinning at constant speed. 
Plot the resulting voltage space vector <code>rotator.y[1] + j*rotator.y[2]</code>.
</p>
</html>"));
    end DemoBLDCVoltages;

    model DemoBLDC "Test example: Brushless DC machine drive"
      extends Modelica.Icons.Example;
      import Modelica.Units.SI;
      import Modelica.Constants.pi;
      constant Integer m=3 "Number of phases";
      parameter SI.Voltage VDC=100*Modelica.Electrical.Polyphase.Functions.factorY2DC(m) "Nominal DC voltage";
      parameter SI.AngularVelocity wNominal(displayUnit="rpm")=2*pi*smpmData.fsNominal/smpmData.p "Nominal speed";
      parameter SI.Torque tauNominal=181.4 "Nominal torque";
      parameter SI.Inertia JLoad=smpmData.Jr "Load inertia";
      Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet
        smpm(
        p=smpmData.p,
        fsNominal=smpmData.fsNominal,
        Rs=smpmData.Rs,
        TsRef=smpmData.TsRef,
        Lszero=smpmData.Lszero,
        Lssigma=smpmData.Lssigma,
        Jr=smpmData.Jr,
        Js=smpmData.Js,
        frictionParameters=smpmData.frictionParameters,
        phiMechanical(fixed=true),
        wMechanical(fixed=true),
        statorCoreParameters=smpmData.statorCoreParameters,
        strayLoadParameters=smpmData.strayLoadParameters,
        VsOpenCircuit=smpmData.VsOpenCircuit,
        Lmd=smpmData.Lmd,
        Lmq=smpmData.Lmq,
        useDamperCage=smpmData.useDamperCage,
        Lrsigmad=smpmData.Lrsigmad,
        Lrsigmaq=smpmData.Lrsigmaq,
        Rrd=smpmData.Rrd,
        Rrq=smpmData.Rrq,
        TrRef=smpmData.TrRef,
        permanentMagnetLossParameters=smpmData.permanentMagnetLossParameters,
        TsOperational=293.15,
        alpha20s=smpmData.alpha20s,
        TrOperational=293.15,
        alpha20r=smpmData.alpha20r)
        annotation (Placement(transformation(extent={{0,-50},{20,-30}})));

      Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
            transformation(
            origin={-10,40},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Mechanics.Rotational.Components.Inertia loadInertia(J=JLoad)
        annotation (Placement(transformation(extent={{40,-50},{60,-30}})));
      Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(m=m,
          terminalConnection="Y")
        annotation (Placement(transformation(extent={{0,-34},{20,-14}})));
      parameter
        Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData
        smpmData(useDamperCage=false, VsOpenCircuit=100) "Synchronous machine data"
        annotation (Placement(transformation(extent={{0,-80},{20,-60}})));
      BLDC.Sensors.HallSensor hallSensor(p=smpmData.p, m=m) annotation (
          Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={30,-60})));
      Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter(m=m)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={10,20})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(extent={{20,40},{0,60}})));
      Utilities.ElectronicCommutator electronicCommutator
        annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
      Modelica.Blocks.Sources.Ramp voltageRamp(
        height=VDC,
        duration=0.5,
        offset=0,
        startTime=0.1)
        annotation (Placement(transformation(extent={{-30,70},{-10,90}})));
      Modelica.Mechanics.Rotational.Sources.TorqueStep loadTorque(
        stepTorque=-tauNominal,
        offsetTorque=0,
        startTime=1)
        annotation (Placement(transformation(extent={{90,-50},{70,-30}})));
      Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentRMSSensor
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={10,-10})));
      Modelica.Blocks.Sources.BooleanConstant booleanConstant
        annotation (Placement(transformation(extent={{-70,20},{-50,40}})));
    initial equation
      smpm.is[1:2] = zeros(2);
    equation
      connect(terminalBox.plug_sn, smpm.plug_sn) annotation (Line(
          points={{4,-30},{4,-30}},
          color={0,0,255}));
      connect(terminalBox.plug_sp, smpm.plug_sp) annotation (Line(
          points={{16,-30},{16,-30}},
          color={0,0,255}));
      connect(smpm.flange, loadInertia.flange_a) annotation (Line(
          points={{20,-40},{40,-40}}));
      connect(smpm.flange, hallSensor.flange)
        annotation (Line(points={{20,-40},{30,-40},{30,-50}}, color={0,0,0}));
      connect(signalVoltage.n, ground.p)
        annotation (Line(points={{0,50},{0,40}},     color={0,0,255}));
      connect(ground.p, inverter.dc_n)
        annotation (Line(points={{0,40},{4,40},{4,30}},    color={0,0,255}));
      connect(signalVoltage.p, inverter.dc_p)
        annotation (Line(points={{20,50},{20,40},{16,40},{16,30}},
                                                                 color={0,0,255}));
      connect(electronicCommutator.fire_p, inverter.fire_p)
        annotation (Line(points={{-9,26},{-2,26}},   color={255,0,255}));
      connect(hallSensor.yC, electronicCommutator.uC) annotation (Line(points={{30,-71},
              {30,-90},{-20,-90},{-20,8}},                color={255,0,255}));
      connect(voltageRamp.y, signalVoltage.v)
        annotation (Line(points={{-9,80},{10,80},{10,62}},color={0,0,127}));
      connect(loadTorque.flange, loadInertia.flange_b)
        annotation (Line(points={{70,-40},{60,-40}}, color={0,0,0}));
      connect(electronicCommutator.fire_n, inverter.fire_n)
        annotation (Line(points={{-9,14},{-2,14}},   color={255,0,255}));
      connect(inverter.ac, currentRMSSensor.plug_p)
        annotation (Line(points={{10,10},{10,0}},
                                                color={0,0,255}));
      connect(currentRMSSensor.plug_n, terminalBox.plugSupply)
        annotation (Line(points={{10,-20},{10,-28}},
                                                   color={0,0,255}));
      connect(booleanConstant.y, electronicCommutator.pwm) annotation (Line(points=
             {{-49,30},{-40,30},{-40,26},{-32,26}}, color={255,0,255}));
      annotation (experiment(
          StopTime=1.5,
          Interval=1e-05,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"),                                      Documentation(
            info="<html>
<p>
A permanent magnet synchronous machine is fed from a variable DC voltage source. 
The fire signals of the inverter bridge are determined from the output signals of the hall sensor. 
Thus the currents are electronically commutated from one phase to the next. 
After the machine ha accelerated, a load torque step is applied.
</p>
<p>
Plot the machine's speed <code>smpm.wMechanical</code>, the electrical torque <code>smpm.tauElectrical</code> and the quasi-RMS current <code>quasiRMSsensor.I</code>. 
Note that - if the reference voltage reaches <code>VDC</code> - full constant DC voltage without switching (except the commutation) is applied.
</p>
<h4>Note:</h4>
<p>
The spatial distribution of the magnetic field of the permanent magnet rotor is sinusoidal. 
Therefore the induced voltage is sinusoidal w.r.t. to time. 
A machine design with rectangular spatial field distribution leads to trapezoidal induced voltage. 
Such a design might be better suited for usage as brushless DC machine.
However, such a machine model is not yet available.
</p>
</html>"));
    end DemoBLDC;

    model DemoBLDCwithPWM "Test example: Brushless DC machine drive"
      extends Modelica.Icons.Example;
      import Modelica.Units.SI;
      import Modelica.Constants.pi;
      constant Integer m=3 "Number of phases";
      parameter SI.Voltage VDC=100*Modelica.Electrical.Polyphase.Functions.factorY2DC(m) "Nominal DC voltage";
      parameter SI.AngularVelocity wNominal(displayUnit="rpm")=2*pi*smpmData.fsNominal/smpmData.p "Nominal speed";
      parameter SI.Torque tauNominal=181.4 "Nominal torque";
      parameter SI.Inertia JLoad=smpmData.Jr "Load inertia";
      Modelica.Electrical.Machines.BasicMachines.SynchronousMachines.SM_PermanentMagnet
        smpm(
        p=smpmData.p,
        fsNominal=smpmData.fsNominal,
        Rs=smpmData.Rs,
        TsRef=smpmData.TsRef,
        Lszero=smpmData.Lszero,
        Lssigma=smpmData.Lssigma,
        Jr=smpmData.Jr,
        Js=smpmData.Js,
        frictionParameters=smpmData.frictionParameters,
        phiMechanical(fixed=true),
        wMechanical(fixed=true),
        statorCoreParameters=smpmData.statorCoreParameters,
        strayLoadParameters=smpmData.strayLoadParameters,
        VsOpenCircuit=smpmData.VsOpenCircuit,
        Lmd=smpmData.Lmd,
        Lmq=smpmData.Lmq,
        useDamperCage=smpmData.useDamperCage,
        Lrsigmad=smpmData.Lrsigmad,
        Lrsigmaq=smpmData.Lrsigmaq,
        Rrd=smpmData.Rrd,
        Rrq=smpmData.Rrq,
        TrRef=smpmData.TrRef,
        permanentMagnetLossParameters=smpmData.permanentMagnetLossParameters,
        TsOperational=293.15,
        alpha20s=smpmData.alpha20s,
        TrOperational=293.15,
        alpha20r=smpmData.alpha20r)
        annotation (Placement(transformation(extent={{0,-50},{20,-30}})));

      Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
            transformation(
            origin={-10,40},
            extent={{-10,-10},{10,10}},
            rotation=270)));
      Modelica.Mechanics.Rotational.Components.Inertia loadInertia(J=JLoad)
        annotation (Placement(transformation(extent={{40,-50},{60,-30}})));
      Modelica.Electrical.Machines.Utilities.TerminalBox terminalBox(m=m,
          terminalConnection="Y")
        annotation (Placement(transformation(extent={{0,-34},{20,-14}})));
      parameter
        Modelica.Electrical.Machines.Utilities.ParameterRecords.SM_PermanentMagnetData
        smpmData(useDamperCage=false, VsOpenCircuit=100) "Synchronous machine data"
        annotation (Placement(transformation(extent={{0,-80},{20,-60}})));
      BLDC.Sensors.HallSensor hallSensor(p=smpmData.p, m=m) annotation (
          Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={30,-60})));
      Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter(m=m)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={10,20})));
      Modelica.Electrical.Analog.Sources.ConstantVoltage
                                                       constantVoltage(V=VDC)
        annotation (Placement(transformation(extent={{20,40},{0,60}})));
      Utilities.ElectronicCommutator electronicCommutator
        annotation (Placement(transformation(extent={{-30,10},{-10,30}})));
      Modelica.Blocks.Sources.Ramp voltageRamp(
        height=VDC,
        duration=0.5,
        offset=0,
        startTime=0.1)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-100,40})));
      Modelica.Mechanics.Rotational.Sources.TorqueStep loadTorque(
        stepTorque=-tauNominal,
        offsetTorque=0,
        startTime=1)
        annotation (Placement(transformation(extent={{90,-50},{70,-30}})));
      Modelica.Electrical.Machines.Sensors.CurrentQuasiRMSSensor currentRMSSensor
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=90,
            origin={10,-10})));
      Modelica.Electrical.PowerConverters.DCDC.Control.SignalPWM pwm(
          useConstantDutyCycle=false) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-50,20})));
      Modelica.Electrical.PowerConverters.DCDC.Control.Voltage2DutyCycle adaptor(
          VLim=VDC)
        annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
    initial equation
      smpm.is[1:2] = zeros(2);
    equation
      connect(terminalBox.plug_sn, smpm.plug_sn) annotation (Line(
          points={{4,-30},{4,-30}},
          color={0,0,255}));
      connect(terminalBox.plug_sp, smpm.plug_sp) annotation (Line(
          points={{16,-30},{16,-30}},
          color={0,0,255}));
      connect(smpm.flange, loadInertia.flange_a) annotation (Line(
          points={{20,-40},{40,-40}}));
      connect(smpm.flange, hallSensor.flange)
        annotation (Line(points={{20,-40},{30,-40},{30,-50}}, color={0,0,0}));
      connect(constantVoltage.n, ground.p)
        annotation (Line(points={{0,50},{0,40}}, color={0,0,255}));
      connect(ground.p, inverter.dc_n)
        annotation (Line(points={{0,40},{4,40},{4,30}},    color={0,0,255}));
      connect(constantVoltage.p, inverter.dc_p) annotation (Line(points={{20,50},{20,
              40},{16,40},{16,30}}, color={0,0,255}));
      connect(electronicCommutator.fire_p, inverter.fire_p)
        annotation (Line(points={{-9,26},{-2,26}},   color={255,0,255}));
      connect(hallSensor.yC, electronicCommutator.uC) annotation (Line(points={{30,-71},
              {30,-90},{-20,-90},{-20,8}},                color={255,0,255}));
      connect(loadTorque.flange, loadInertia.flange_b)
        annotation (Line(points={{70,-40},{60,-40}}, color={0,0,0}));
      connect(electronicCommutator.fire_n, inverter.fire_n)
        annotation (Line(points={{-9,14},{-2,14}},   color={255,0,255}));
      connect(inverter.ac, currentRMSSensor.plug_p)
        annotation (Line(points={{10,10},{10,0}},
                                                color={0,0,255}));
      connect(currentRMSSensor.plug_n, terminalBox.plugSupply)
        annotation (Line(points={{10,-20},{10,-28}},
                                                   color={0,0,255}));
      connect(electronicCommutator.pwm, pwm.fire)
        annotation (Line(points={{-32,26},{-39,26}}, color={255,0,255}));
      connect(adaptor.dutyCycle, pwm.dutyCycle)
        annotation (Line(points={{-59,40},{-50,40},{-50,32}}, color={0,0,127}));
      connect(voltageRamp.y, adaptor.v)
        annotation (Line(points={{-89,40},{-82,40}}, color={0,0,127}));
      annotation (experiment(
          StopTime=1.5,
          Interval=1e-05,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"), Documentation(
            info="<html>
<p>
A permanent magnet synchronous machine is fed from a constant DC voltage source. 
The fire signals of the inverter bridge are determined from the output signals of the hall sensor. 
Thus the currents are electronically commutated from one phase to the next. 
The magnitude of the voltage applied to the machine is controlled by the PWM signal. 
After the machine ha accelerated, a load torque step is applied.
</p>
<p>
Plot the machine's speed <code>smpm.wMechanical</code>, the electrical torque <code>smpm.tauElectrical</code> and the quasi-RMS current <code>quasiRMSsensor.I</code>.
</p>
<h4>Note:</h4>
<p>
The spatial distribution of the magnetic field of the permanent magnet rotor is sinusoidal. 
Therefore the induced voltage is sinusoidal w.r.t. to time. 
A machine design with rectangular spatial field distribution leads to trapezoidal induced voltage. 
Such a design might be better suited for usage as brushless DC machine.
However, such a machine model is not yet available.
</p>
</html>"),
        Diagram(coordinateSystem(extent={{-120,-100},{100,100}})),
        Icon(coordinateSystem(extent={{-100,-100},{100,100}})));
    end DemoBLDCwithPWM;
  end Examples;

  package CommonBlocks "Blocks planned to move to Modelica.Blocks"
    extends Modelica.Icons.Package;
    block IntervalTest
      "Output y is true, if input u is within the specified interval"
      extends Modelica.Blocks.Interfaces.partialBooleanThresholdComparison(final threshold=0);
      parameter Real lowerLimit "Lower limit of interval";
      parameter Boolean ClosedOnLeft=false "Include lower limit?" annotation(Evaluate=true);
      parameter Real upperLimit "Upper limit of interval";
      parameter Boolean ClosedOnRight=false "Include upper limit?" annotation(Evaluate=true);
      parameter Boolean InsideInterval=true "u inside interval?" annotation(Evaluate=true);
    equation
      if ClosedOnLeft then
        if ClosedOnRight then
          if InsideInterval then
            y = u>=lowerLimit and u<=upperLimit;
          else
            y = u< lowerLimit or  u> upperLimit;
          end if;
        else
          if InsideInterval then
            y = u>=lowerLimit and u< upperLimit;
          else
            y = u< lowerLimit or  u>=upperLimit;
          end if;
        end if;
      else
        if ClosedOnRight then
          if InsideInterval then
            y = u> lowerLimit and u<=upperLimit;
          else
            y = u<=lowerLimit or  u> upperLimit;
          end if;
        else
          if InsideInterval then
            y = u> lowerLimit and u< upperLimit;
          else
            y = u<=lowerLimit or  u>=upperLimit;
          end if;
        end if;
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                {100,100}}), graphics={Text(
              extent={{-60,50},{60,-30}},
              textColor={0,0,0},
              textString="[ ? ]")}),
                                Documentation(info="<html>
<p>
The output is <strong>true</strong> if the Real input is within interval specified by lower and upper limit.
The Boolean parameters ClosedOnLeft and ClosedOnRight indicate whether lower respectively upper limit are included in the interval. 
If Boolean parameter InsideInterval = false, the output is inverted (u is outside the interval).
</p>
</html>"));
    end IntervalTest;

    block UnwrapAngle "Angle tracking observer"
      extends Modelica.Blocks.Interfaces.SISO(u(final unit="rad", displayUnit="deg"),
        y(final unit="rad", displayUnit="deg"));
      Modelica.Blocks.Interfaces.RealOutput w(final unit="rad/s", displayUnit="rpm")
        "Angular velocity"
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
      parameter Modelica.Units.SI.Time Ti=1e-6 "Integral time constant of controller";
      parameter Modelica.Units.SI.Angle phi0=0 "Initial angle";
      Modelica.Blocks.Math.Cos cos1
        annotation (Placement(transformation(extent={{-70,-30},{-50,-10}})));
      Modelica.Blocks.Math.Sin sin1
        annotation (Placement(transformation(extent={{-70,10},{-50,30}})));
      Modelica.Electrical.Machines.SpacePhasors.Blocks.Rotator rotator
        annotation (Placement(transformation(extent={{-30,10},{-10,-10}})));
      Modelica.Blocks.Continuous.Integrator integralController(
        k=1,
        initType=Modelica.Blocks.Types.Init.InitialOutput,
        y_start=phi0)
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      Modelica.Blocks.Math.Gain gain(k=1/Ti)
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
    equation
      connect(u, sin1.u) annotation (Line(points={{-120,0},{-80,0},{-80,20},{-72,20}},
            color={0,0,127}));
      connect(u, cos1.u) annotation (Line(points={{-120,0},{-80,0},{-80,-20},{-72,-20}},
            color={0,0,127}));
      connect(integralController.y,rotator. angle) annotation (Line(points={{61,0},{
              80,0},{80,20},{-20,20},{-20,12}}, color={0,0,127}));
      connect(gain.y,integralController. u)
        annotation (Line(points={{21,0},{38,0}}, color={0,0,127}));
      connect(rotator.y[2],gain. u)
        annotation (Line(points={{-9,0},{-2,0}}, color={0,0,127}));
      connect(sin1.y, rotator.u[2]) annotation (Line(points={{-49,20},{-40,20},{-40,
              0},{-32,0}}, color={0,0,127}));
      connect(cos1.y, rotator.u[1]) annotation (Line(points={{-49,-20},{-40,-20},{-40,
              0},{-32,0}}, color={0,0,127}));
      connect(integralController.y, y)
        annotation (Line(points={{61,0},{110,0}}, color={0,0,127}));
      connect(gain.y, w) annotation (Line(points={{21,0},{30,0},{30,-60},{110,-60}},
            color={0,0,127}));
      annotation (Icon(graphics={
            Text(
              extent={{58,-40},{100,-80}},
              textColor={28,108,200},
              textString="w"),
            Polygon(
              points={{0,90},{-8,68},{8,68},{0,90}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{0,-80},{0,68}}, color={192,192,192}),
            Polygon(
              points={{90,0},{68,8},{68,-8},{90,0}},
              lineColor={192,192,192},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-90,0},{68,0}}, color={192,192,192}),
            Line(points={{-72,20},{-72,20},{-40,60},{-40,-40},{40,60},{40,-60},{56,-40}}),
            Line(points={{56,80},{-72,-80}}, color={28,108,200},
              thickness=0.5)}),
          Documentation(info="<html>
<p>
An angle tracking observer is a very robust method to determine the angle of a space phasor. 
If we calculate <code>cos</code> and <code>sin</code> of a wrapped angle - no matter whether in the interval [0, 2&pi;) or (-&pi;, +&pi;] - 
we can use this algorithm to unwrap the angle.
</p>
<p>
Rotating the space phasor by an angle that is determined by the controller - whose goal is to bring the imaginary part to zero - the result is the desired continuos angle. 
The result can be differentiated to obtain the angular velocity, but as a bonus the input of the integral controller already is the angular velocity. 
The result approximates the desired angle by a firstOrder whose time constant is the integral time constant:
</p>
<p>
<code>Im(e<sup>j(&phi;-&phi;')</sup>)=sin(&phi;-&phi;')</code> which can be approximated by <code>(&phi;-&phi;')</code>
</p>
<p>
Using an integral contoller, the transfer function of the closed loop can be determined as:
<code>&phi;'=&phi;/(1 + s*T<sub>I</sub>)</code>.
</p>
</html>"));
    end UnwrapAngle;
    annotation (Icon(graphics={
        Line(
          origin={-51.25,0},
          points={{21.25,-35.0},{-13.75,-35.0},{-13.75,35.0},{6.25,35.0}}),
        Polygon(
          origin={-40,35},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{10.0,0.0},{-5.0,5.0},{-5.0,-5.0}}),
        Line(
          origin={51.25,0},
          points={{-21.25,35.0},{13.75,35.0},{13.75,-35.0},{-6.25,-35.0}}),
        Polygon(
          origin={40,-35},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-10.0,0.0},{5.0,5.0},{5.0,-5.0}}),
        Rectangle(
          origin={0,-34.851},
          fillColor={255,255,255},
          extent={{-30.0,-20.1488},{30.0,20.1488}}),
        Rectangle(
          origin={0,35.1488},
          fillColor={255,255,255},
          extent={{-30.0,-20.1488},{30.0,20.1488}})}));
  end CommonBlocks;

  package Sensors "Different types of speed and angle sensors"
    extends Modelica.Icons.SensorsPackage;

    model HallSensor "Hall sensor"
      extends
        Modelica.Mechanics.Rotational.Interfaces.PartialOneFlangeAndSupport;
      import Modelica.Units.SI;
      import Modelica.Constants.pi;
      parameter Integer p(final min=1, start=2) "Number of pole pairs";
      parameter Integer m(min=3) = 3 "Number of stator phases";
      parameter SI.Angle orientation[m]=
        Modelica.Electrical.Polyphase.Functions.symmetricOrientation(m) "Orientation of phases";
      parameter SI.Angle phi0=0 "Initial mechanical angle (zero position)";
      Modelica.Blocks.Interfaces.RealOutput y(final unit="rad", displayUnit="deg")
        "Electrical angle"
        annotation (Placement(transformation(extent={{-100,50},{-120,70}})));
      Modelica.Blocks.Interfaces.BooleanOutput yC[m] "Commutation signals"
        annotation (Placement(transformation(extent={{-100,-10},{-120,10}})));
      Modelica.Mechanics.Rotational.Sensors.RelAngleSensor relAngleSensor
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={80,0})));
      Modelica.Blocks.Math.WrapAngle wrapAngle
        annotation (Placement(transformation(extent={{20,-10},{0,10}})));
      Modelica.Blocks.Math.Add add(k1=p, k2=p)
        annotation (Placement(transformation(extent={{50,-10},{30,10}})));
      Modelica.Blocks.Sources.Constant const(k=phi0)
        annotation (Placement(transformation(extent={{10,10},{-10,-10}},
            rotation=270,
            origin={60,-30})));
      Modelica.Blocks.Routing.Replicator replicator(nout=m)
        annotation (Placement(transformation(extent={{-20,-10},{-40,10}})));
      CommonBlocks.IntervalTest intervalTest[m](
        lowerLimit=lowerLimit,
        upperLimit=upperLimit,
        InsideInterval=InsideInterval)
        annotation (Placement(transformation(extent={{-50,-10},{-70,10}})));
    protected
      parameter SI.Angle loLim[m]={Modelica.Math.wrapAngle(-orientation[k] - pi/2, wrapAngle.positiveRange) for k in 1:m};
      parameter SI.Angle upLim[m]={Modelica.Math.wrapAngle(-orientation[k] + pi/2, wrapAngle.positiveRange) for k in 1:m};
      parameter SI.Angle lowerLimit[m]={if loLim[k]<upLim[k] then loLim[k] else upLim[k] for k in 1:m};
      parameter SI.Angle upperLimit[m]={if loLim[k]<upLim[k] then upLim[k] else loLim[k] for k in 1:m};
      parameter Boolean InsideInterval[m]={loLim[k]<upLim[k] for k in 1:m};
    equation
      connect(relAngleSensor.flange_a, internalSupport)
        annotation (Line(points={{80,-10},{80,-80},{0,-80}}, color={0,0,0}));
      connect(relAngleSensor.flange_b, flange)
        annotation (Line(points={{80,10},{90,10},{90,0},{100,0}}, color={0,0,0}));
      connect(wrapAngle.y, y)
        annotation (Line(points={{-1,0},{-10,0},{-10,60},{-110,60}},
                                                    color={0,0,127}));
      connect(add.u2, const.y)
        annotation (Line(points={{52,-6},{60,-6},{60,-19}}, color={0,0,127}));
      connect(relAngleSensor.phi_rel, add.u1)
        annotation (Line(points={{69,0},{60,0},{60,6},{52,6}}, color={0,0,127}));
      connect(add.y, wrapAngle.u)
        annotation (Line(points={{29,0},{22,0}}, color={0,0,127}));
      connect(wrapAngle.y, replicator.u)
        annotation (Line(points={{-1,0},{-18,0}}, color={0,0,127}));
      connect(replicator.y, intervalTest.u)
        annotation (Line(points={{-41,0},{-48,0}}, color={0,0,127}));
      connect(intervalTest.y, yC)
        annotation (Line(points={{-71,0},{-110,0}}, color={255,0,255}));
       annotation (
        Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(extent={{-70,70},{70,-70}}, lineColor={95,95,95},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Line(points={{0,0},{0,70}},  color={95,95,95}),
            Line(points={{0,-70},{0,0}}, color={95,95,95}),
            Line(points={{-36,60},{-100,60}},
                                            color={95,95,95}),
            Line(points={{100,0},{70,0}}, color={95,95,95}),
            Text(
              extent={{-150,120},{150,80}},
              textColor={0,0,255},
              fillColor={255,255,255},
              textString="%name"),
            Line(points={{0,-30},{-0.545517,38.9449}},
                                         color={95,95,95},
              origin={-26,15},
              rotation=60),
            Line(points={{0,-30},{-0.545517,38.9449}},
                                         color={95,95,95},
              origin={34,-19},
              rotation=60),
            Line(points={{0,-30},{0.545517,38.9449}},
                                         color={95,95,95},
              origin={26,15},
              rotation=-60),
            Line(points={{0,-30},{0.545517,38.9449}},
                                         color={95,95,95},
              origin={-34,-19},
              rotation=-60),
            Ellipse(extent={{-20,20},{20,-20}}, lineColor={95,95,95},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-30,-10},{30,-70}},
              textColor={64,64,64},
              textString="rad"),
            Line(points={{-100,0},{-70,0}},   color={255,0,255})}),
        Documentation(info="<html>
<p>
Simple model of a hall sensor, i.e. measuring the angle of the flange (w.r.t. the optional support), multiplying by the number of pole pairs p to obtain the electrical angle,
and adding a correction term i.e. the initial angle of the flange phi0.
</p>
<p>
Additionally, the Boolean commutation signals for <code>m</code> phases are generated, 
i.e. an ouput <code>yC[k] = true</code> when the angle is within [-pi/2, +pi/2] of the orientation of phase k.
</p>
</html>"));
    end HallSensor;

    model SinCosResolver "Sin/Cos-Resolver"
      extends
        Modelica.Mechanics.Rotational.Interfaces.PartialOneFlangeAndSupport;
      import Modelica.Constants.pi;
      parameter Integer p(final min=1, start=2) "Number of pole pairs";
      parameter Real offset=1.5 "Offset of output";
      parameter Real amplitude=1 "Amplitude of output";
      parameter Modelica.Units.SI.Angle phi0=-pi/p "Initial mechanical angle (zero position)";
      Modelica.Mechanics.Rotational.Sensors.RelAngleSensor relAngleSensor
        annotation (Placement(transformation(extent={{10,-10},{-10,10}},
            rotation=270,
            origin={80,0})));
      Modelica.Blocks.Math.Add add(k1=p, k2=p)
        annotation (Placement(transformation(extent={{50,-10},{30,10}})));
      Modelica.Blocks.Sources.Constant const(k=phi0)
        annotation (Placement(transformation(extent={{10,10},{-10,-10}},
            rotation=270,
            origin={60,-30})));
      Modelica.Blocks.Math.Cos cos1
        annotation (Placement(transformation(extent={{10,-50},{-10,-30}})));
      Modelica.Blocks.Math.Sin sin1
        annotation (Placement(transformation(extent={{10,30},{-10,50}})));
      Modelica.Blocks.Interfaces.RealOutput y[4] "Resolver signals"
        annotation (Placement(transformation(extent={{-100,-10},{-120,10}}),
            iconTransformation(extent={{-100,-10},{-120,10}})));
      Modelica.Blocks.Math.Add add1(k1=+amplitude)
        annotation (Placement(transformation(extent={{-40,-50},{-60,-70}})));
      Modelica.Blocks.Math.Add addOffset2(k1=-amplitude)
        annotation (Placement(transformation(extent={{-40,-30},{-60,-10}})));
      Modelica.Blocks.Math.Add addOffset3(k1=amplitude)
        annotation (Placement(transformation(extent={{-40,30},{-60,10}})));
      Modelica.Blocks.Math.Add addOffset4(k1=-amplitude)
        annotation (Placement(transformation(extent={{-40,50},{-60,70}})));
      Modelica.Blocks.Sources.Constant constOffset(k=offset)
        annotation (Placement(transformation(extent={{10,-10},{-10,10}})));
    equation
      connect(add.u1, relAngleSensor.phi_rel)
        annotation (Line(points={{52,6},{60,6},{60,0},{69,0}}, color={0,0,127}));
      connect(const.y, add.u2)
        annotation (Line(points={{60,-19},{60,-6},{52,-6}}, color={0,0,127}));
      connect(add.y, sin1.u)
        annotation (Line(points={{29,0},{20,0},{20,40},{12,40}},color={0,0,127}));
      connect(add.y, cos1.u)
        annotation (Line(points={{29,0},{20,0},{20,-40},{12,-40}},color={0,0,127}));
      connect(addOffset4.y, y[4]) annotation (Line(points={{-61,60},{-80,60},{
              -80,7.5},{-110,7.5}},
                           color={0,0,127}));
      connect(addOffset3.y, y[3]) annotation (Line(points={{-61,20},{-70,20},{
              -70,2.5},{-110,2.5}},
                           color={0,0,127}));
      connect(addOffset2.y, y[2]) annotation (Line(points={{-61,-20},{-70,-20},
              {-70,-2.5},{-110,-2.5}},
                            color={0,0,127}));
      connect(add1.y, y[1]) annotation (Line(points={{-61,-60},{-80,-60},{-80,
              -7.5},{-110,-7.5}},
                            color={0,0,127}));
      connect(internalSupport, relAngleSensor.flange_a)
        annotation (Line(points={{0,-80},{80,-80},{80,-10}}, color={0,0,0}));
      connect(sin1.y, addOffset4.u1) annotation (Line(points={{-11,40},{-20,40},
              {-20,66},{-38,66}}, color={0,0,127}));
      connect(sin1.y, addOffset3.u1) annotation (Line(points={{-11,40},{-20,40},
              {-20,14},{-38,14}}, color={0,0,127}));
      connect(cos1.y, addOffset2.u1) annotation (Line(points={{-11,-40},{-20,
              -40},{-20,-14},{-38,-14}}, color={0,0,127}));
      connect(cos1.y, add1.u1) annotation (Line(points={{-11,-40},{-20,-40},{
              -20,-66},{-38,-66}}, color={0,0,127}));
      connect(constOffset.y, addOffset4.u2) annotation (Line(points={{-11,0},{
              -30,0},{-30,54},{-38,54}}, color={0,0,127}));
      connect(constOffset.y, addOffset3.u2) annotation (Line(points={{-11,0},{
              -30,0},{-30,26},{-38,26}}, color={0,0,127}));
      connect(constOffset.y, addOffset2.u2) annotation (Line(points={{-11,0},{
              -30,0},{-30,-26},{-38,-26}}, color={0,0,127}));
      connect(constOffset.y, add1.u2) annotation (Line(points={{-11,0},{-30,0},
              {-30,-54},{-38,-54}}, color={0,0,127}));
      connect(relAngleSensor.flange_b, flange) annotation (Line(points={{80,10},
              {90,10},{90,0},{100,0}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(extent={{-80,80},{80,-80}}, lineColor={95,95,95},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
        Text(textColor={0,0,255},
          extent={{-120,-20},{120,20}},
          textString="%name",
              origin={0,120},
              rotation=180),
            Line(points={{-80,0},{-100,0}},color={0,0,0}),
            Line(points={{80,0},{100,0}}, color={0,0,0}),
            Ellipse(extent={{-20,20},{20,-20}}, lineColor={95,95,95},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-80,80},{-74.4,78.1},{-68.7,72.3},{-63.1,63},{-56.7,48.7},{-48.6,
                  26.6},{-29.3,-32.5},{-22.1,-51.7},{-15.7,-65.3},{-10.1,-73.8},{-4.42,
                  -78.8},{1.21,-79.9},{6.83,-77.1},{12.5,-70.6},{18.1,-60.6},{24.5,-45.7},
                  {32.6,-23},{50.3,31.3},{57.5,50.7},{63.9,64.6},{69.5,73.4},{75.2,78.6},
                  {80,80}},
              smooth=Smooth.Bezier),
            Line(
              points={{-80,0},{-68.7,34.2},{-61.5,53.1},{-55.1,66.4},{-49.4,74.6},{-43.8,
                  79.1},{-38.2,79.8},{-32.6,76.6},{-26.9,69.7},{-21.3,59.4},{-14.9,44.1},
                  {-6.83,21.2},{10.1,-30.8},{17.3,-50.2},{23.7,-64.2},{29.3,-73.1},{35,
                  -78.4},{40.6,-80},{46.2,-77.6},{51.9,-71.5},{57.5,-61.9},{63.9,-47.2},
                  {72,-24.8},{80,0}},
              smooth=Smooth.Bezier)}),                               Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<p>
This is a model of sin/cos-encoder.
It senses the angle of the flange <code>phi</code> (w.r.t. the optional support), providing 4 analog signals:
</p>
<ul>
<li><code>y[1] = offset + amplitude*cos(p*(phi - phi0))</code></li>
<li><code>y[2] = offset - amplitude*cos(p*(phi - phi0))</code></li>
<li><code>y[3] = offset + amplitude*sin(p*(phi - phi0))</code></li>
<li><code>y[4] = offset - amplitude*sin(p*(phi - phi0))</code></li>
</ul>
<p>
It is used to measure angular velocity and rotor angle of electric drives. 
The offset of the output is used to sense whether the sensor is working or defect. 
It can easily be removed by subtracting <code>y[1] - y[2]</code> and <code>y[3] - y[4]</code>.
</p>
<h4>Note:</h4>
<p>
Usually the number of pole pairs <code>p</code> matches the number of pole pairs of the permanent magnet synchronous machine. 
The sensor directly provides the electrical rotor angle, i.e. <code>p*phi<sub>Mechancial</sub></code>, and the electrial angular velocity, i.e. <code>p*w<sub>Mechancial</sub></code>, 
if block <a href=\"modelica://BLDC.Utilities.SinCosEvaluation\">SinCosEvaluation</a> is used.
</p>
<p>
Note that phi0 has to be set that way, that in shaft position phi0 the flux linkage of phase 1 is a maximum.
</p>
</html>"));
    end SinCosResolver;

    model IncrementalEncoder "Incremental encoder"
      extends
        Modelica.Mechanics.Rotational.Interfaces.PartialOneFlangeAndSupport;
      import Modelica.Constants.pi;
      parameter Integer pRev(final min=1, start=128) "Pulses per revolution";
      Modelica.Blocks.Interfaces.BooleanOutput y[3] "Encoder signals"
        annotation (Placement(transformation(extent={{-100,-10},{-120,10}})));
      Modelica.Mechanics.Rotational.Sensors.RelAngleSensor relAngleSensor
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=270,
            origin={80,0})));
      Modelica.Blocks.Math.WrapAngle wrapAngle1(positiveRange=true)
        annotation (Placement(transformation(extent={{60,-10},{40,10}})));
      Modelica.Blocks.Math.Gain gain(k=pRev)
        annotation (Placement(transformation(extent={{20,-10},{0,10}})));
      CommonBlocks.IntervalTest intervalTest1(
        lowerLimit=0,
        upperLimit=2*pi/pRev/4,
        InsideInterval=true)
        annotation (Placement(transformation(extent={{-60,-40},{-80,-20}})));
      CommonBlocks.IntervalTest intervalTest2(
        lowerLimit=pi/2,
        upperLimit=3*pi/2,
        InsideInterval=false)
        annotation (Placement(transformation(extent={{-60,-10},{-80,10}})));
      CommonBlocks.IntervalTest intervalTest3(
        lowerLimit=0,
        upperLimit=pi,
        InsideInterval=true)
        annotation (Placement(transformation(extent={{-60,20},{-80,40}})));
      Modelica.Blocks.Math.WrapAngle wrapAngle2(positiveRange=true)
        annotation (Placement(transformation(extent={{-10,-10},{-30,10}})));
    equation
      connect(relAngleSensor.flange_b, flange)
        annotation (Line(points={{80,10},{100,10},{100,0}}, color={0,0,0}));
      connect(internalSupport, relAngleSensor.flange_a) annotation (Line(points={{0,-80},
              {0,-60},{80,-60},{80,-10}}, color={0,0,0}));
      connect(relAngleSensor.phi_rel, wrapAngle1.u)
        annotation (Line(points={{69,0},{62,0}}, color={0,0,127}));
      connect(wrapAngle1.y, gain.u)
        annotation (Line(points={{39,0},{22,0}}, color={0,0,127}));
      connect(intervalTest2.y, y[2])
        annotation (Line(points={{-81,0},{-110,0}}, color={255,0,255}));
      connect(intervalTest3.y, y[3]) annotation (Line(points={{-81,30},{-90,30},{-90,
              6.66667},{-110,6.66667}}, color={255,0,255}));
      connect(intervalTest1.y, y[1]) annotation (Line(points={{-81,-30},{-90,-30},{-90,
              -6.66667},{-110,-6.66667}}, color={255,0,255}));
      connect(wrapAngle1.y, intervalTest1.u) annotation (Line(points={{39,0},{30,
              0},{30,-30},{-58,-30}}, color={0,0,127}));
      connect(gain.y, wrapAngle2.u)
        annotation (Line(points={{-1,0},{-8,0}}, color={0,0,127}));
      connect(wrapAngle2.y, intervalTest2.u)
        annotation (Line(points={{-31,0},{-58,0}}, color={0,0,127}));
      connect(wrapAngle2.y, intervalTest3.u) annotation (Line(points={{-31,0},{
              -50,0},{-50,30},{-58,30}}, color={0,0,127}));
      annotation (Icon(graphics={
            Line(points={{-70,0},{-100,0}}, color={95,95,95}),
            Line(points={{100,0},{70,0}}, color={95,95,95}),
            Text(
              extent={{-150,120},{150,80}},
              textColor={0,0,255},
              fillColor={255,255,255},
              textString="%name"),
            Ellipse(extent={{-80,80},{80,-80}}, lineColor={95,95,95},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Ellipse(extent={{-20,20},{20,-20}}, lineColor={95,95,95},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,-40},{-60,-40},{-60,-20},{-20,-20},{-20,-40},{20,
                  -40},{20,-20},{60,-20},{60,-40},{80,-40}},
                                            color={0,0,0}),
            Line(points={{80,20},{40,20},{40,40},{0,40},{0,20},{-40,20},{-40,40},
                  {-80,40},{-80,20}},      color={0,0,0})}),
                                                Documentation(info="<html>
<p>
This is a model of an incremental encoder.  
It senses the angle of the flange <code>phi</code> (w.r.t. the optional support), providing 3 Boolean signals:
</p>
<ul>
<li>0-track <code>y[1]</code> = 1 pulse per revolution with a length of <code>2*pi/(4*pRev)</code></li>
<li>A-track <code>y[2]</code> = <code>pRev</code> pulses per revolution</li>
<li>B-track <code>y[3]</code> = <code>pRev</code> pulses per revolution, displaced by <code>2*pi/(4*pRev)</code></li>
</ul>
<p>
It is used to measure angular velocity of electric drives, using block <a href=\"modelica://BLDC.Utilities.EncoderEvaluation\">EncoderEvaluation</a> 
or <a href=\"modelica://BLDC.Utilities.EncoderPulseCount\">EncoderPulseCount</a>.
</p>
</html>"));
    end IncrementalEncoder;

  end Sensors;

  package Utilities "Utility blocks"
    extends Modelica.Icons.UtilitiesPackage;

    block ElectronicCommutator "Fire signals for a BLDC bridge"
      extends Modelica.Blocks.Icons.BooleanBlock;
      constant Integer m(min=3) = 3 "Number of stator phases";
      Modelica.Blocks.Interfaces.BooleanInput pwm
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Modelica.Blocks.Interfaces.BooleanInput uC[m] "Commutation signals"
        annotation (Placement(transformation(extent={{-20,-20},{20,20}},
            rotation=90,
            origin={0,-120})));
      Modelica.Blocks.Interfaces.BooleanOutput fire_p[m]
        "Fire signals of positive potential transistors"
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
      Modelica.Blocks.Interfaces.BooleanOutput fire_n[m]
        "Fire signals of negative potential transistors"
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
    protected
      constant Integer I1[6]={2,1,1,3,2,3} "Indices of on states in sectors";
      constant Integer I0[6]={3,2,3,1,1,2} "Indices of off states in sectors";
      constant Integer Ix[6]={1,3,2,2,3,1} "Indices of open states in sectors";
      //Hall 100 110 010 011 001 101
      //Fire x10 1x0 10x x01 0x1 01x
      Integer c=sum({if uC[k] then integer(2^(k - 1)) else 0 for k in 1:m}) "Sector code";
    algorithm
      assert(c>=1 and c<=6, "Hall sensor sector error!");
      fire_p[I1[c]]:=pwm;
      fire_n[I1[c]]:=not pwm;
      fire_p[I0[c]]:=not pwm;
      fire_n[I0[c]]:=pwm;
      fire_p[Ix[c]]:=false;
      fire_n[Ix[c]]:=false;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Line(points={{-60,30},{-60,50},{0,50},{0,30},{60,30},{60,50}}, color={255,
                  0,255}),
            Line(points={{-60,-10},{-20,-10},{-20,10},{40,10},{40,-10},{60,-10}},
                color={255,0,255}),
            Line(points={{-60,-30},{-40,-30},{-40,-50},{20,-50},{20,-30},{60,-30}},
                color={255,0,255})}),                                Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<p>
According to the Hall sensor signals <code>uC[m]</code>, the inactive (open) leg is choossen 
as well as the two legs that apply the DC voltage to two terminals of the machine, 
and therefore let a current flow through the corresponding phases of the machine 
to obtain a current space phasor perpendicular to the rotor's magnetic field. 
Of course due to the block commutation the angle between rotor position and current space phasor 
varies in the range <code>[&#177;90&deg; - 30&deg;, &#177;90&deg; + 30&deg;]</code>.
</p>
<p>
Setting the input <code>pwm = true</code> chooses the position of the current space phasor as <code>+90&deg;</code>, whereas 
setting the input <code>pwm = false</code> results in a position of the current space phasor as <code>-90&deg;</code> w.r.t. the rotor position. 
Applying a pwm signal (as for a brushed DC machine) to the input <code>pwm</code> offers to set the mean voltage applied to the phases choosen according to the Hall signals.
</p>
</html>"));
    end ElectronicCommutator;

    block SinCosEvaluation "Evaluation of Sin/Cos-resolver signals"
      extends Modelica.Blocks.Icons.Block;
      Modelica.Blocks.Interfaces.RealInput u[4] "Resolver signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Blocks.Interfaces.RealOutput phi(final unit="rad", displayUnit="deg")
        "Electrical angle"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.RealOutput w(final unit="rad/s", displayUnit="rpm")
        "Electrical angular velocity"
        annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
      parameter Modelica.Units.SI.Time Ti=1e-6 "Integral time constant of controller";
      parameter Modelica.Units.SI.Angle phi0=0 "Initial mechanical angle (zero position)";
      Modelica.Blocks.Math.Feedback feedbackCos
        annotation (Placement(transformation(extent={{-70,-20},{-50,-40}})));
      Modelica.Blocks.Math.Feedback feedbackSin
        annotation (Placement(transformation(extent={{-70,20},{-50,40}})));
      Modelica.Electrical.Machines.SpacePhasors.Blocks.Rotator rotator
        annotation (Placement(transformation(extent={{-30,10},{-10,-10}})));
      Modelica.Blocks.Continuous.Integrator integralController(
        k=1,
        initType=Modelica.Blocks.Types.Init.InitialOutput,
        y_start=phi0)
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      Modelica.Blocks.Math.Gain gain(k=1/Ti)
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
    equation
      connect(u[1], feedbackCos.u1) annotation (Line(points={{-120,-15},{-80,-15},
              {-80,-30},{-68,-30}}, color={0,0,127}));
      connect(u[2], feedbackCos.u2) annotation (Line(points={{-120,-5},{-60,-5},
              {-60,-22}}, color={0,0,127}));
      connect(u[3], feedbackSin.u1) annotation (Line(points={{-120,5},{-80,5},{-80,
              30},{-68,30}}, color={0,0,127}));
      connect(u[4], feedbackSin.u2) annotation (Line(points={{-120,15},{-60,15},
              {-60,22}}, color={0,0,127}));
      connect(feedbackCos.y, rotator.u[1]) annotation (Line(points={{-51,-30},{-40,
              -30},{-40,0},{-32,0}}, color={0,0,127}));
      connect(rotator.u[2], feedbackSin.y) annotation (Line(points={{-32,0},{-40,
              0},{-40,30},{-51,30}}, color={0,0,127}));
      connect(integralController.y, rotator.angle) annotation (Line(points={{61,0},{
              70,0},{70,20},{-20,20},{-20,12}}, color={0,0,127}));
      connect(integralController.y, phi)
        annotation (Line(points={{61,0},{110,0}}, color={0,0,127}));
      connect(gain.y, integralController.u)
        annotation (Line(points={{21,0},{38,0}}, color={0,0,127}));
      connect(rotator.y[2], gain.u)
        annotation (Line(points={{-9,0},{-2,0}}, color={0,0,127}));
      connect(integralController.u, w) annotation (Line(points={{38,0},{30,0},{
              30,-60},{110,-60}}, color={0,0,127}));
      annotation (Documentation(info="<html>
<p>
Evaluates the input signals from a <a href=\"modelica://BLDC.Sensors.SinCosResolver\">SinCosResolver</a>:
</p>
<ul>
<li><code>y[1] = offset + cos(p*phi)</code></li>
<li><code>y[2] = offset - cos(p*phi)</code></li>
<li><code>y[3] = offset + sin(p*phi)</code></li>
<li><code>y[4] = offset - sin(p*phi)</code></li>
</ul>
<p>
to obtain the electrical rotor angle <code>phi</code> and the electrial angular velocity <code>w</code>:
</p>
<ul>
<li>phi = angle (continuous)</li>
<li>w   = angular velocity</li>
</ul>
<p>
Subtracting the inputs pairwise  <code>y[1] - y[2]</code> and <code>y[3] - y[4]</code> eliminates the offset. 
The results are interpreted as real and imaginary part of a space phasor. 
Calculating the angle of the space phasor using <code>atan2</code> gives the desired rotor angle, wrapped to the interval [0, 2*p). 
This calculation is sensitive on amplitude errors of the input signal, and the result cannot be differentiated.
<p>
</p>
An alternative algorithm with better stability and robustness is a tracking obeserver: 
The space phasor is rotated (Park-transform) by an angle that is determined by a fast integral controller bringing the imaginary part of the rotated phasor to zero. 
Differentiating the output gives the angular velocity.
</p>
<h4>Note:</h4>
<p>
Usually the number of pole pairs <code>p</code> matches the number of pole pairs of the permanent magnet synchronous machine. 
Therefore the results directly provide the electrical rotor angle, i.e. <code>p*phi<sub>Mechancial</sub></code>, and the electrial angular velocity, i.e. <code>p*w<sub>Mechancial</sub></code>.
</p>
</html>"),     Icon(graphics={
            Line(
              points={{-80,80},{-74.4,78.1},{-68.7,72.3},{-63.1,63},{-56.7,48.7},{
                  -48.6,26.6},{-29.3,-32.5},{-22.1,-51.7},{-15.7,-65.3},{-10.1,-73.8},
                  {-4.42,-78.8},{1.21,-79.9},{6.83,-77.1},{12.5,-70.6},{18.1,-60.6},
                  {24.5,-45.7},{32.6,-23},{50.3,31.3},{57.5,50.7},{63.9,64.6},{69.5,
                  73.4},{75.2,78.6},{80,80}},
              smooth=Smooth.Bezier),
            Line(
              points={{-80,0},{-68.7,34.2},{-61.5,53.1},{-55.1,66.4},{-49.4,74.6},{
                  -43.8,79.1},{-38.2,79.8},{-32.6,76.6},{-26.9,69.7},{-21.3,59.4},{
                  -14.9,44.1},{-6.83,21.2},{10.1,-30.8},{17.3,-50.2},{23.7,-64.2},{
                  29.3,-73.1},{35,-78.4},{40.6,-80},{46.2,-77.6},{51.9,-71.5},{57.5,
                  -61.9},{63.9,-47.2},{72,-24.8},{80,0}},
              smooth=Smooth.Bezier),
            Text(
              extent={{60,-60},{100,-100}},
              textColor={0,0,0},
              textString="w")}));
    end SinCosEvaluation;

    block EncoderEvaluation "Evaluation of incremental encoder signals"
      extends Modelica.Blocks.Icons.DiscreteBlock;
      import Modelica.Constants.pi;
      import Modelica.Constants.eps;
      parameter Integer pRev(final min=1, start=128) "Pulses per revolution";
      parameter Modelica.Units.SI.Angle phi0=0 "Initial mechanical angle (zero position)";
      Modelica.Blocks.Interfaces.BooleanInput u[3] "Encoder signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Blocks.Interfaces.RealOutput w(final unit="rad/s", displayUnit="rpm")
        "Mechanical anglar velocity"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.RealOutput phi(final unit="rad", displayUnit="deg")
        "Mechanical angle"
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
    protected
      Boolean A=u[2];
      Boolean B=u[3];
      Boolean notA=not A;
      Boolean notB=not B;
      Boolean firstEdge;
      Modelica.Units.SI.Time t0;
    initial equation
      pre(A)=true;
      pre(B)=false;
      pre(notA)=false;
      pre(notB)=true;
      firstEdge=true;
      t0=time;
      phi=phi0;
    equation
      der(phi)= w;
      when edge(A) then
        w=(if notB then 1 else -1)*(if noEvent(time - pre(t0) < eps) or pre(firstEdge) then 0 else 2*pi/(4*pRev)/(time - pre(t0)));
        firstEdge=false;
        t0=time;
      elsewhen edge(notA) then
        w=(if B then 1 else -1)*(if noEvent(time - pre(t0) < eps) or pre(firstEdge) then 0 else 2*pi/(4*pRev)/(time - pre(t0)));
        firstEdge=false;
        t0= time;
      elsewhen edge(B) then
        w=(if A then 1 else -1)*(if noEvent(time - pre(t0) < eps) or pre(firstEdge) then 0 else 2*pi/(4*pRev)/(time - pre(t0)));
        firstEdge=false;
        t0= time;
      elsewhen edge(notB) then
        w=(if notA then 1 else -1)*(if noEvent(time - pre(t0) < eps) or pre(firstEdge) then 0 else 2*pi/(4*pRev)/(time - pre(t0)));
        firstEdge=false;
        t0= time;
      end when;
      annotation (Documentation(info="<html>
<p>Evaluates the input signals from an <a href=\"modelica://BLDC.Sensors.IncrementalEncoder\">IncrementalEncoder</a>:</p>
<ul>
<li>0-track <code>y[1]</code> = 1 pulse per revolution with a length of <code>2*pi/(4*pRev)</code></li>
<li>A-track <code>y[2]</code> = <code>pRev</code> pulses per revolution</li>
<li>B-track <code>y[3]</code> = <code>pRev</code> pulses per revolution, displaced by <code>2*pi/(4*pRev)</code></li>
</ul>
<p>
and determines mechanical angular velocity <code>w</code> and mechanical angle <code>phi</code>:
</p>
<ul>
<li>w    = angular velocity</li>
<li>phi  = angle = integral of w</li>
</ul>
<p>
The time span from one edge of the input signals is measured, thus calculating the speed of rotation: <code>2*pi/(4*pRev)/(time - t0)</code>.
Taking both rising and falling edges of A-track and B-track into account, <code>4*pRev</code> edges per revolution are observed.
Note that the output is discrete w.r.t. to time, it is not differentiable.
The algorithm depends on interrupts (which appear in Modelica as when-clauses). 
The direction of rotation has to be detected by a logic from the values of A-track and B-track. 
Note that this algorithm depends on the accuracy of the representation of time, 
but usually gives better results than counting edges during a time gate.
</p>
<p>
Note that phi0 has to be set that way, that in shaft position phi0 the flux linkage of phase 1 is a maximum.
</p>
</html>"),     Icon(graphics={
            Line(points={{-80,20},{-60,20},{-60,40},{-20,40},{-20,20},{20,20},{20,40},
                  {60,40},{60,20},{80,20}}, color={0,0,0}),
            Line(points={{-80,-40},{-40,-40},{-40,-20},{0,-20},{0,-40},{40,-40},{40,-20},
                  {80,-20},{80,-40}},      color={0,0,0}),
            Text(
              extent={{60,20},{100,-20}},
              textColor={0,0,0},
              textString="w"),
            Line(
              points={{-60,80},{-60,40}},
              color={0,0,0},
              pattern=LinePattern.Dash),
            Line(
              points={{-40,80},{-40,-20}},
              color={0,0,0},
              pattern=LinePattern.Dash),
            Line(points={{-80,70},{-20,70}},color={0,0,0}),
            Polygon(
              points={{3,0},{-11,4},{-11,-4},{3,0}},
              lineColor={0,0,0},
              fillColor={235,235,235},
              fillPattern=FillPattern.Solid,
              origin={-63,70},
              rotation=360),
            Polygon(
              points={{3,0},{-11,4},{-11,-4},{3,0}},
              lineColor={0,0,0},
              fillColor={235,235,235},
              fillPattern=FillPattern.Solid,
              origin={-37,70},
              rotation=180)}));
    end EncoderEvaluation;

    block EncoderPulseCount "Count pulses of an incremental"
      extends Modelica.Blocks.Interfaces.DiscreteBlock(samplePeriod=0.01);
      import Modelica.Constants.pi;
      parameter Integer pRev(final min=1, start=128) "Pulses per revolution";
      parameter Modelica.Units.SI.Angle phi0=0 "Initial mechanical angle (zero position)";
      Modelica.Blocks.Interfaces.BooleanInput u[3] "Encoder signals"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Blocks.Interfaces.RealOutput w(final unit="rad/s", displayUnit="rpm")
        "Mechanical angular velocity"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.RealOutput phi(final unit="rad", displayUnit="deg")
        "Mechanical angle"
        annotation (Placement(transformation(extent={{100,50},{120,70}})));
    protected
      Boolean A=u[2];
      Boolean B=u[3];
      Boolean notA=not A;
      Boolean notB=not B;
      Integer count;
    initial equation
      pre(A)=true;
      pre(B)=false;
      pre(notA)=false;
      pre(notB)=true;
      count=0;
      phi=phi0;
    equation
      der(phi)= w;
    algorithm
      when edge(A) then
        count:=pre(count) + (if notB then 1 else -1);
      elsewhen edge(notA) then
        count:=pre(count) + (if B then 1 else -1);
      elsewhen edge(B) then
        count:=pre(count) + (if A then 1 else -1);
      elsewhen edge(notB) then
        count:=pre(count) + (if notA then 1 else -1);
      end when;
      when sampleTrigger then
        w:=count*2*pi/(4*pRev)/samplePeriod;
        count:=0;
      end when;
      annotation (Documentation(info="<html>
<p>Evaluates the input signals from an <a href=\"modelica://BLDC.Sensors.IncrementalEncoder\">IncrementalEncoder</a>:</p>
<ul>
<li>0-track <code>y[1]</code> = 1 pulse per revolution with a length of <code>2*pi/(4*pRev)</code></li>
<li>A-track <code>y[2]</code> = <code>pRev</code> pulses per revolution</li>
<li>B-track <code>y[3]</code> = <code>pRev</code> pulses per revolution, displaced by <code>2*pi/(4*pRev)</code></li>
</ul>
<p>
and determines mechanical angular velocity <code>w</code>.
</p>
<p>
During the samplePeriod all rising and falling edges of A-track and B-track are counted. 
The direction of rotation has to be detected by a logic from the values of A-track and B-track. 
From that count, the angular velocity can be calculated: <code>w=count*2*pi(4*pRev)/samplePeriod</code>. 
The accuracy of the result depends on the number of pulses per revolution and the samplePeriod. 
Note that the output is discrete w.r.t. to time, it is not differentiable.
The algorithm depends on interrupts (which appear in Modelica as when-clauses).
</p>
</html>"),     Icon(graphics={
            Line(points={{-80,20},{-60,20},{-60,40},{-20,40},{-20,20},{20,20},{20,40},
                  {60,40},{60,20},{80,20}}, color={0,0,0}),
            Line(points={{-80,-40},{-40,-40},{-40,-20},{0,-20},{0,-40},{40,-40},{40,-20},
                  {80,-20},{80,-40}},      color={0,0,0}),
            Text(
              extent={{60,20},{100,-20}},
              textColor={0,0,0},
              textString="w"),
            Line(
              points={{-70,80},{-70,60}},
              color={0,0,0},
              pattern=LinePattern.Dash),
            Line(points={{-70,70},{70,70}}, color={0,0,0}),
            Line(
              points={{70,80},{70,60}},
              color={0,0,0},
              pattern=LinePattern.Dash),
            Polygon(
              points={{3,0},{-11,4},{-11,-4},{3,0}},
              lineColor={0,0,0},
              fillColor={235,235,235},
              fillPattern=FillPattern.Solid,
              origin={-67,70},
              rotation=180),
            Polygon(
              points={{3,0},{-11,4},{-11,-4},{3,0}},
              lineColor={0,0,0},
              fillColor={235,235,235},
              fillPattern=FillPattern.Solid,
              origin={67,70},
              rotation=360)}));
    end EncoderPulseCount;

  end Utilities;
  annotation (version="1.0.0", versionDate="2021-12-10",
  uses(Modelica(version="4.0.0"), ModelicaServices(version="4.0.0")));
end BLDC;
