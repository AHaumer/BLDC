within BLDC.Examples;
model DemoElectronicCommutator
  "Test example: Demonstrate the electronic commutator"
  extends Modelica.Icons.Example;
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  constant Integer m=3 "Number of phases";
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
  Sensors.SinCosResolver sinCosResolver(p=p, phi0=0)
    annotation (Placement(transformation(extent={{-30,20},{-50,40}})));
  Utilities.SinCosEvaluation sinCosEvaluation
    annotation (Placement(transformation(extent={{-20,20},{0,40}})));
  Modelica.Blocks.Math.WrapAngle wrapAngle(positiveRange=true)
    annotation (Placement(transformation(extent={{10,20},{30,40}})));
  BLDC.Utilities.ElectronicCommutator3phase electronicCommutator3phase annotation (
    Placement(visible = true, transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(constantSpeed.flange, hallSensor.flange) annotation (
    Line(points = {{-70, 0}, {-50, 0}}, color = {0, 0, 0}));
  connect(constantSpeed.flange, sinCosResolver.flange) annotation (
    Line(points = {{-70, 0}, {-60, 0}, {-60, 30}, {-50, 30}}, color = {0, 0, 0}));
  connect(sinCosResolver.y, sinCosEvaluation.u) annotation (
    Line(points = {{-29, 30}, {-22, 30}}, color = {0, 0, 127}));
  connect(sinCosEvaluation.phi, wrapAngle.u) annotation (
    Line(points = {{1, 30}, {8, 30}}, color = {0, 0, 127}));
  connect(hallSensor.yC, electronicCommutator3phase.uC) annotation (
    Line(points = {{-28, 0}, {-20, 0}, {-20, -20}, {20, -20}, {20, -12}}, color = {255, 0, 255}, thickness = 0.5));
  annotation (experiment(
      StopTime=1,
      Interval=1e-05,
      Tolerance=1e-06), Documentation(info="<html>
<p>
Demonstrates how the rotor position is determined by the electronic commutator:
Compare <code>wrapAngle.y</code> and protected variable <code>electronicCommutator.rotorPosition</code>.
</p>
</html>"));
end DemoElectronicCommutator;
