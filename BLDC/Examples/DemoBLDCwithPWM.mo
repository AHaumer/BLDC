within BLDC.Examples;
model DemoBLDCwithPWM "Test example: Brushless DC machine drive"
  extends Modelica.Icons.Example;
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  parameter SI.Voltage VDC=smpmData.VsNominal*Modelica.Electrical.Polyphase.Functions.factorY2DC(smpmData.m) "Nominal DC voltage";
  parameter SI.Voltage VDCmax=1.1*VDC "max. DC voltage";
  parameter SI.Frequency fS=1000 "Switching frequency";
  parameter SI.Inertia JLoad=smpmData.Jr "Load inertia";
  Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousMachines.SM_PermanentMagnet
    smpm(
    m=smpmData.m,
    p=smpmData.p,
    TsOperational=smpmData.TsNominal,
    TrOperational=smpmData.TrNominal,
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
    alpha20s=smpmData.alpha20s,
    alpha20r=smpmData.alpha20r)
    annotation (Placement(transformation(extent={{10,-50},{30,-30}})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        origin={0,40},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.Rotational.Components.Inertia loadInertia(J=JLoad)
    annotation (Placement(transformation(extent={{50,-50},{70,-30}})));
  Modelica.Electrical.Machines.Utilities.MultiTerminalBox terminalBox(m=smpmData.m,
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{10,-34},{30,-14}})));
  parameter ParameterRecords.SmpmData smpmData      "Synchronous machine data"
    annotation (Placement(transformation(extent={{10,-80},{30,-60}})));
  BLDC.Sensors.HallSensor hallSensor(p=smpmData.p, m=smpmData.m)
    annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={40,-60})));
  Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter(m=smpmData.m)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,20})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDCmax)
    annotation (Placement(transformation(extent={{30,60},{10,80}})));
  Utilities.ElectronicCommutator electronicCommutator(m=smpmData.m,
    useConstantPWM=false)
    annotation (Placement(transformation(extent={{-20,10},{0,30}})));
  Modelica.Blocks.Sources.Ramp voltageRamp(
    height=VDC,
    duration=1,
    offset=0,
    startTime=0.1)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-90,40})));
  Modelica.Mechanics.Rotational.Sources.TorqueStep loadTorque(
    stepTorque=-smpmData.tauNominal,
    offsetTorque=0,
    startTime=1.5)
    annotation (Placement(transformation(extent={{100,-50},{80,-30}})));
  Modelica.Electrical.Polyphase.Sensors.CurrentQuasiRMSSensor currentRMSSensor(m=
        smpmData.m)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={20,-10})));
  FromMSL.SignalPWM pwm(
      useConstantDutyCycle=false,
      f=fS,
      refType=BLDC.FromMSL.SingleReferenceType.Triangle)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-40,20})));
  Modelica.Electrical.PowerConverters.DCDC.Control.Voltage2DutyCycle adaptor(VLim=VDCmax)
    annotation (Placement(transformation(extent={{-70,30},{-50,50}})));
  Modelica.Electrical.Polyphase.Basic.Star star(m=smpmData.mSystems) annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-40})));
  Modelica.Electrical.Analog.Sensors.CurrentSensor currentSensor annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={30,50})));
initial equation
  smpm.is[1:2] = zeros(2);
equation
  connect(terminalBox.plug_sn, smpm.plug_sn) annotation (Line(
      points={{14,-30},{14,-30}},
      color={0,0,255}));
  connect(terminalBox.plug_sp, smpm.plug_sp) annotation (Line(
      points={{26,-30},{26,-30}},
      color={0,0,255}));
  connect(smpm.flange, loadInertia.flange_a) annotation (Line(
      points={{30,-40},{50,-40}}));
  connect(smpm.flange, hallSensor.flange)
    annotation (Line(points={{30,-40},{40,-40},{40,-50}}, color={0,0,0}));
  connect(constantVoltage.n, ground.p)
    annotation (Line(points={{10,70},{10,40}},
                                             color={0,0,255}));
  connect(ground.p, inverter.dc_n)
    annotation (Line(points={{10,40},{14,40},{14,30}}, color={0,0,255}));
  connect(electronicCommutator.fire_p, inverter.fire_p)
    annotation (Line(points={{1,26},{8,26}},   color={255,0,255}));
  connect(hallSensor.yC, electronicCommutator.uC) annotation (Line(points={{40,-71},
          {40,-90},{-10,-90},{-10,8}},      color={255,0,255}));
  connect(loadTorque.flange, loadInertia.flange_b)
    annotation (Line(points={{80,-40},{70,-40}}, color={0,0,0}));
  connect(electronicCommutator.fire_n, inverter.fire_n)
    annotation (Line(points={{1,14},{8,14}},   color={255,0,255}));
  connect(inverter.ac, currentRMSSensor.plug_p)
    annotation (Line(points={{20,10},{20,0}},
                                            color={0,0,255}));
  connect(currentRMSSensor.plug_n, terminalBox.plugSupply)
    annotation (Line(points={{20,-20},{20,-28}},
                                               color={0,0,255}));
  connect(electronicCommutator.pwm, pwm.fire)
    annotation (Line(points={{-22,26},{-29,26}}, color={255,0,255}));
  connect(adaptor.dutyCycle, pwm.dutyCycle)
    annotation (Line(points={{-49,40},{-40,40},{-40,32}}, color={0,0,127}));
  connect(voltageRamp.y, adaptor.v)
    annotation (Line(points={{-79,40},{-72,40}}, color={0,0,127}));
  connect(star.plug_p, terminalBox.starpoint)
    annotation (Line(points={{0,-30},{0,-28},{10,-28}}, color={0,0,255}));
  connect(constantVoltage.p, currentSensor.p)
    annotation (Line(points={{30,70},{30,60}}, color={0,0,255}));
  connect(currentSensor.n, inverter.dc_p)
    annotation (Line(points={{30,40},{26,40},{26,30}}, color={0,0,255}));
  annotation (experiment(
      StopTime=2,
      Interval=1e-05,
      Tolerance=1e-06), Documentation(info="<html>
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
The maximum DC voltage is set 10% higher than the theoretical DC volatge, thus the duty cacle is limited slightly below 1 
to avoid constant switching state of the transistors but to keep pulse voltage.
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
    Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
    Icon(coordinateSystem(extent={{-100,-100},{100,100}})));
end DemoBLDCwithPWM;
