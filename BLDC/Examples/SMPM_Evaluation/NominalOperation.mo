within BLDC.Examples.SMPM_Evaluation;
model NominalOperation "PermanentMagnetSynchronousMachine fed by current source"
  extends Modelica.Icons.Example;
  Modelica.Magnetic.FundamentalWave.BasicMachines.SynchronousMachines.SM_PermanentMagnet
    smpm(
    m=smpmData.m,
    p=smpmData.p,
    fsNominal=smpmData.fsNominal,
    TsOperational=smpmData.TsNominal,
    TrOperational=smpmData.TrNominal,
    Rs=smpmData.Rs,
    TsRef=smpmData.TsRef,
    alpha20s=smpmData.alpha20s,
    Lszero=smpmData.Lszero,
    Lssigma=smpmData.Lssigma,
    Jr=smpmData.Jr,
    Js=smpmData.Js,
    frictionParameters=smpmData.frictionParameters,
    phiMechanical(fixed=true),
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
    alpha20r=smpmData.alpha20r,
    permanentMagnetLossParameters=smpmData.permanentMagnetLossParameters)
    annotation (Placement(transformation(extent={{-20,-30},{0,-10}})));
  Modelica.Electrical.Polyphase.Sources.SignalCurrent signalCurrent(final m=
        smpmData.m)
                 annotation (Placement(transformation(
        origin={-10,30},
        extent={{-10,-10},{10,10}},
        rotation=270)));
  Modelica.Electrical.Polyphase.Basic.Star star(final m=smpmData.m)
                                                           annotation (
      Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-60,30})));
  Modelica.Electrical.Machines.Utilities.DQToThreePhase dqToThreePhase(m=
        smpmData.m, p=smpmData.p)
    annotation (Placement(transformation(extent={{30,20},{10,40}})));
  Modelica.Blocks.Sources.Constant iq(k=smpmData.IsNominal)
    annotation (Placement(transformation(extent={{70,0},{50,20}})));
  Modelica.Blocks.Sources.Constant id(k=0)
    annotation (Placement(transformation(extent={{70,40},{50,60}})));
  Modelica.Electrical.Polyphase.Sensors.VoltageQuasiRMSSensor voltageQuasiRMSSensor(m=
        smpmData.m)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-30,30})));
  Modelica.Electrical.Analog.Basic.Ground groundM annotation (Placement(
        transformation(
        origin={-60,-20},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Modelica.Electrical.Machines.Utilities.MultiTerminalBox
                                                     terminalBox(m=smpmData.m,
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{-20,-14},{0,6}})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={20,0})));
  Modelica.Mechanics.Rotational.Sensors.MultiSensor multiSensorMechanical
    annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={40,-20})));
  Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed=
        smpmData.wNominal)
    annotation (Placement(transformation(extent={{80,-30},{60,-10}})));
  parameter ParameterRecords.Smpm1FT7102
                                      smpmData
    "Synchronous machine data"
    annotation (Placement(transformation(extent={{-20,-60},{0,-40}})));
  Modelica.Electrical.Polyphase.Basic.Star starM(m=smpmData.mSystems)
                                                                     annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-40,-8})));
equation
  connect(terminalBox.plug_sn, smpm.plug_sn) annotation (Line(
      points={{-16,-10},{-16,-10}},
      color={0,0,255}));
  connect(terminalBox.plug_sp, smpm.plug_sp) annotation (Line(
      points={{-4,-10},{-4,-10}},
      color={0,0,255}));
  connect(id.y, dqToThreePhase.d) annotation (Line(points={{49,50},{40,50},{40,
          36},{32,36}},           color={0,0,127}));
  connect(iq.y, dqToThreePhase.q) annotation (Line(points={{49,10},{40,10},{40,
          24},{32,24}},           color={0,0,127}));
  connect(smpm.flange, multiSensorMechanical.flange_a)
    annotation (Line(points={{0,-20},{30,-20}}));
  connect(dqToThreePhase.y, signalCurrent.i)
    annotation (Line(points={{9,30},{2,30}},     color={0,0,127}));
  connect(smpm.flange, angleSensor.flange)
    annotation (Line(points={{0,-20},{20,-20},{20,-10}}, color={0,0,0}));
  connect(angleSensor.phi, dqToThreePhase.phi) annotation (Line(points={{20,11},
          {20,18}},                          color={0,0,127}));
  connect(multiSensorMechanical.flange_b, constantSpeed.flange)
    annotation (Line(points={{50,-20},{60,-20}}, color={0,0,0}));
  connect(groundM.p, star.pin_n)
    annotation (Line(points={{-60,-10},{-60,20}}, color={0,0,255}));
  connect(terminalBox.plugSupply, signalCurrent.plug_n)
    annotation (Line(points={{-10,-8},{-10,20}}, color={0,0,255}));
  connect(star.plug_p, signalCurrent.plug_p) annotation (Line(points={{-60,40},{
          -60,50},{-10,50},{-10,40}}, color={0,0,255}));
  connect(star.plug_p, voltageQuasiRMSSensor.plug_n) annotation (Line(points={{
          -60,40},{-60,50},{-30,50},{-30,40}}, color={0,0,255}));
  connect(terminalBox.plugSupply, voltageQuasiRMSSensor.plug_p) annotation (Line(
        points={{-10,-8},{-10,10},{-30,10},{-30,20}}, color={0,0,255}));
  connect(terminalBox.starpoint, starM.plug_p)
    annotation (Line(points={{-20,-8},{-30,-8}}, color={0,0,255}));
  connect(groundM.p, starM.pin_n)
    annotation (Line(points={{-60,-10},{-60,-8},{-50,-8}}, color={0,0,255}));
  annotation (experiment(StopTime=0.1, Interval=1E-4, Tolerance=1e-06), Documentation(
        info="<html>
<p>
A synchronous machine with permanent magnets is fed by nominal q-current at nominal speed, 
to verify nominal torque and nominal voltage.
</p>
</html>"));
end NominalOperation;
