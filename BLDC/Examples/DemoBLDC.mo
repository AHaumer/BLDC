within BLDC.Examples;
model DemoBLDC "Test example: Brushless DC machine drive"
  extends Modelica.Icons.Example;
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  parameter SI.Voltage VDC=smpmData.VsOpenCircuit*Modelica.Electrical.Polyphase.Functions.factorY2DC(smpmData.ms) "Nominal DC voltage";
  parameter SI.AngularVelocity wNominal(displayUnit="rpm")=2*pi*smpmData.fsNominal/smpmData.p "Nominal speed";
  parameter SI.Torque tauNominal=181.4 "Nominal torque";
  parameter SI.Inertia JLoad=smpmData.Jr "Load inertia";
  Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousMachines.SM_PermanentMagnet
    smpm(
    m=smpmData.ms,
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
    annotation (Placement(transformation(extent={{10,-50},{30,-30}})));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (Placement(
        transformation(
        origin={0,40},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Mechanics.Rotational.Components.Inertia loadInertia(J=JLoad)
    annotation (Placement(transformation(extent={{50,-50},{70,-30}})));
  Modelica.Electrical.Machines.Utilities.MultiTerminalBox terminalBox(m=smpmData.ms,
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{10,-34},{30,-14}})));
  parameter Utilities.SM_PermanentMagnetData smpmData "Synchronous machine data"
    annotation (Placement(transformation(extent={{10,-80},{30,-60}})));
  BLDC.Sensors.HallSensor hallSensor(p=smpmData.p, m=smpmData.ms)
                                                        annotation (Placement(
        transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={40,-60})));
  Modelica.Electrical.PowerConverters.DCAC.Polyphase2Level inverter(m=smpmData.ms)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,20})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
    annotation (Placement(transformation(extent={{30,40},{10,60}})));
  Utilities.ElectronicCommutator electronicCommutator(m=smpmData.ms)
    annotation (Placement(transformation(extent={{-20,10},{0,30}})));
  Modelica.Blocks.Sources.Ramp voltageRamp(
    height=VDC,
    duration=1,
    offset=0,
    startTime=0.1)
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
  Modelica.Mechanics.Rotational.Sources.TorqueStep loadTorque(
    stepTorque=-tauNominal,
    offsetTorque=0,
    startTime=1.5)
    annotation (Placement(transformation(extent={{100,-50},{80,-30}})));
  Modelica.Electrical.Polyphase.Sensors.CurrentQuasiRMSSensor currentRMSSensor(m=
        smpmData.ms)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={20,-10})));
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
  connect(signalVoltage.n, ground.p)
    annotation (Line(points={{10,50},{10,40}},   color={0,0,255}));
  connect(ground.p, inverter.dc_n)
    annotation (Line(points={{10,40},{14,40},{14,30}}, color={0,0,255}));
  connect(signalVoltage.p, inverter.dc_p)
    annotation (Line(points={{30,50},{30,40},{26,40},{26,30}},
                                                             color={0,0,255}));
  connect(electronicCommutator.fire_p, inverter.fire_p)
    annotation (Line(points={{1,26},{8,26}},   color={255,0,255}));
  connect(hallSensor.yC, electronicCommutator.uC) annotation (Line(points={{40,-71},
          {40,-90},{-10,-90},{-10,8}},      color={255,0,255}));
  connect(voltageRamp.y, signalVoltage.v)
    annotation (Line(points={{-79,40},{-70,40},{-70,80},{20,80},{20,62}},
                                                      color={0,0,127}));
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
  annotation (experiment(
      StopTime=2,
      Interval=1e-05,
      Interval=1e-05,
      Tolerance=1e-06), Documentation(info="<html>
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
