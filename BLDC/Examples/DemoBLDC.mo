within BLDC.Examples;
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
  BLDC.Sensors.HallSensor hallSensor(p=smpmData.p, m=m) annotation (Placement(
        transformation(
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
  Utilities.ElectronicCommutator3phase electronicCommutator
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
    annotation (Line(points={{-9,26},{-2,26}}, color={255,0,255}));
  connect(hallSensor.yC, electronicCommutator.uC) annotation (Line(points={{30,
          -71},{30,-90},{-20,-90},{-20,8}}, color={255,0,255}));
  connect(voltageRamp.y, signalVoltage.v)
    annotation (Line(points={{-9,80},{10,80},{10,62}},color={0,0,127}));
  connect(loadTorque.flange, loadInertia.flange_b)
    annotation (Line(points={{70,-40},{60,-40}}, color={0,0,0}));
  connect(electronicCommutator.fire_n, inverter.fire_n)
    annotation (Line(points={{-9,14},{-2,14}}, color={255,0,255}));
  connect(inverter.ac, currentRMSSensor.plug_p)
    annotation (Line(points={{10,10},{10,0}},
                                            color={0,0,255}));
  connect(currentRMSSensor.plug_n, terminalBox.plugSupply)
    annotation (Line(points={{10,-20},{10,-28}},
                                               color={0,0,255}));
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
