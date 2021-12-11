within BLDC.Examples;
model DemoElectronicCommutator
  "Test example: Demonstrate the electronic commutator"
  extends Modelica.Icons.Example;
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  constant Integer m=6 "Number of phases";
  parameter Integer p(final min=1)=2 "Number of pole pairs";
  parameter SI.Frequency fNominal=50 "Nominal frequqncy";
  parameter SI.AngularVelocity wNominal(displayUnit="rpm")=2*pi*fNominal/p "Nominal speed";
  Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(
    w_fixed=0.1*wNominal,  phi(start=0, fixed=true))
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  BLDC.Sensors.HallSensor hallSensor(p=p, m=m) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-40,0})));
  Utilities.RotorAngle rotorAngle(m=m)
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
  Sensors.SinCosResolver sinCosResolver(p=p, phi0=0)
    annotation (Placement(transformation(extent={{-30,20},{-50,40}})));
  Utilities.SinCosEvaluation sinCosEvaluation
    annotation (Placement(transformation(extent={{-20,20},{0,40}})));
  Modelica.Blocks.Math.WrapAngle wrapAngle
    annotation (Placement(transformation(extent={{10,20},{30,40}})));
equation
  connect(constantSpeed.flange, hallSensor.flange)
    annotation (Line(points={{-70,0},{-50,0}},   color={0,0,0}));
  connect(hallSensor.yC, rotorAngle.uC)
    annotation (Line(points={{-29,0},{-22,0}}, color={255,0,255}));
  connect(constantSpeed.flange, sinCosResolver.flange) annotation (Line(points={{
          -70,0},{-60,0},{-60,30},{-50,30}}, color={0,0,0}));
  connect(sinCosResolver.y, sinCosEvaluation.u)
    annotation (Line(points={{-29,30},{-22,30}}, color={0,0,127}));
  connect(sinCosEvaluation.phi, wrapAngle.u)
    annotation (Line(points={{1,30},{8,30}}, color={0,0,127}));
  annotation (experiment(
      Interval=1e-05,
      Tolerance=1e-06,
      __Dymola_Algorithm="Dassl"),                                      Documentation(
        info="<html>
<p>
Demonstrates how the rotor position is determined by the electronic commutator.
</p>
</html>"));
end DemoElectronicCommutator;
