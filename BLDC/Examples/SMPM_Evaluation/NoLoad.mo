within BLDC.Examples.SMPM_Evaluation;
model NoLoad "No-Load of PermanentMagnetSynchronousMachine"
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
  Modelica.Electrical.Polyphase.Basic.Star star(final m=smpmData.m)
                                                           annotation (
      Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-60,30})));
  Modelica.Electrical.Polyphase.Sensors.VoltageQuasiRMSSensor voltageQuasiRMSSensor(m=
        smpmData.m)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-10,30})));
  Modelica.Electrical.Analog.Basic.Ground groundM annotation (Placement(
        transformation(
        origin={-60,-20},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Modelica.Electrical.Machines.Utilities.MultiTerminalBox
                                                     terminalBox(m=smpmData.m,
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{-20,-14},{0,6}})));
  Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed=
        smpmData.wNominal)
    annotation (Placement(transformation(extent={{40,-30},{20,-10}})));
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
  connect(groundM.p, star.pin_n)
    annotation (Line(points={{-60,-10},{-60,20}}, color={0,0,255}));
  connect(star.plug_p, voltageQuasiRMSSensor.plug_n) annotation (Line(points={{-60,40},
          {-60,50},{-10,50},{-10,40}},         color={0,0,255}));
  connect(terminalBox.plugSupply, voltageQuasiRMSSensor.plug_p) annotation (Line(
        points={{-10,-8},{-10,20}},                   color={0,0,255}));
  connect(terminalBox.starpoint, starM.plug_p)
    annotation (Line(points={{-20,-8},{-30,-8}}, color={0,0,255}));
  connect(groundM.p, starM.pin_n)
    annotation (Line(points={{-60,-10},{-60,-8},{-50,-8}}, color={0,0,255}));
  connect(constantSpeed.flange, smpm.flange)
    annotation (Line(points={{20,-20},{0,-20}}, color={0,0,0}));
  annotation (experiment(StopTime=0.1, Interval=1E-4, Tolerance=1e-06), Documentation(
        info="<html>
<p>
The no-load voltage of a synchronous machine with permanent magnets measured.
</p>
</html>"));
end NoLoad;
