within BLDC.Examples.SMPM_Evaluation;
model LockedRotor
  "Locked-rotor test of PermanentMagnetSynchronousMachine"
  extends Modelica.Icons.Example;
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  import Modelica.Electrical.Machines.Thermal.convertResistance;
  import Modelica.Math.BooleanVectors.countTrue;
  parameter SI.Angle phi(displayUnit="deg")=-pi/2 "Electrical angle of fixation";
  parameter Boolean on[smpmData.m]={true,false,false} "Vector of on states";
  parameter SI.Current IDC=sqrt(2)*smpmData.IsNominal "DC current";
  parameter Integer n_On=countTrue(on);
  parameter Integer nOff=size(on, 1) - n_On;
  parameter SI.Resistance RDC=(1.0/n_On + 1.0/nOff)*
    convertResistance(smpmData.Rs, smpmData.TsRef, smpmData.alpha20s, smpm.TsOperational)
    "Equivalent DC resistance";
  parameter SI.Voltage VDC=RDC*IDC "DC voltage";
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
  Modelica.Electrical.Analog.Basic.Ground groundM annotation (Placement(
        transformation(
        origin={-60,-20},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  Modelica.Electrical.Machines.Utilities.MultiTerminalBox
                                                     terminalBox(m=smpmData.m,
      terminalConnection="Y")
    annotation (Placement(transformation(extent={{-20,-14},{0,6}})));
  Modelica.Mechanics.Rotational.Components.Disc       disc(deltaPhi=phi/smpmData.p)
    annotation (Placement(transformation(extent={{60,-30},{40,-10}})));
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
  Modelica.Mechanics.Rotational.Components.Fixed fixed
    annotation (Placement(transformation(extent={{60,-30},{80,-10}})));
  Modelica.Electrical.Analog.Sources.ConstantVoltage constantVoltage(V=VDC)
    annotation (Placement(transformation(extent={{-20,60},{0,80}})));
  Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor
    annotation (Placement(transformation(extent={{10,-30},{30,-10}})));
  Modelica.Electrical.Polyphase.Sensors.CurrentSensor currentSensor(m=smpmData.m)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-10,10})));
  Modelica.Electrical.Machines.SpacePhasors.Blocks.ToSpacePhasor toSpacePhasor(m=
       smpmData.m)
    annotation (Placement(transformation(extent={{10,0},{30,20}})));
  Utilities.SwitchingState switchingState(m=smpmData.m, on=on)
                                          annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-10,40})));
initial equation
  smpm.is=zeros(smpmData.m);
equation
  connect(terminalBox.plug_sn, smpm.plug_sn) annotation (Line(
      points={{-16,-10},{-16,-10}},
      color={0,0,255}));
  connect(terminalBox.plug_sp, smpm.plug_sp) annotation (Line(
      points={{-4,-10},{-4,-10}},
      color={0,0,255}));
  connect(terminalBox.starpoint, starM.plug_p)
    annotation (Line(points={{-20,-8},{-30,-8}}, color={0,0,255}));
  connect(groundM.p, starM.pin_n)
    annotation (Line(points={{-60,-10},{-60,-8},{-50,-8}}, color={0,0,255}));
  connect(disc.flange_a, fixed.flange)
    annotation (Line(points={{60,-20},{70,-20}}, color={0,0,0}));
  connect(smpm.flange, torqueSensor.flange_a)
    annotation (Line(points={{0,-20},{10,-20}}, color={0,0,0}));
  connect(torqueSensor.flange_b, disc.flange_b)
    annotation (Line(points={{30,-20},{40,-20}}, color={0,0,0}));
  connect(currentSensor.plug_n, terminalBox.plugSupply)
    annotation (Line(points={{-10,0},{-10,-8}}, color={0,0,255}));
  connect(currentSensor.i, toSpacePhasor.u)
    annotation (Line(points={{1,10},{8,10}}, color={0,0,127}));
  connect(constantVoltage.p, switchingState.dc_p) annotation (Line(points={{-20,70},
          {-20,60},{-16,60},{-16,50}}, color={0,0,255}));
  connect(constantVoltage.n, switchingState.dc_n)
    annotation (Line(points={{0,70},{0,60},{-4,60},{-4,50}}, color={0,0,255}));
  connect(switchingState.ac, currentSensor.plug_p)
    annotation (Line(points={{-10,30},{-10,20}}, color={0,0,255}));
  annotation (experiment(StopTime=0.5, Interval=1E-4, Tolerance=1e-06), Documentation(
        info="<html>
<p>
The rotor of a synchronous machine with permanent magnets is locked at a defined angle, 
then a DC voltage is applied such way that nominal current space phasor at angle = 0 is caused. 
Note that torque depends on the angle at which the rotor is fixed.
</p>
<h4>Note:</h4>
<p>Using number of phases m&lt;&gt;3, adapt equivalent DC resistance RDC and vector of on-states.</p>
</html>"));
end LockedRotor;
