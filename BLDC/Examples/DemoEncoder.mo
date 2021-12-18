within BLDC.Examples;
model DemoEncoder "Demonstrate various encoder / resolver models"
  extends Modelica.Icons.Example;
  import Modelica.Constants.pi;
  constant Integer m=3 "Number of phases";
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
    annotation (Placement(transformation(extent={{-20,70},{-40,90}})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(extent={{-20,50},{-40,70}})));
  Sensors.SinCosResolver sinCosResolver(p=p, phi0=phi0)
    annotation (Placement(transformation(extent={{30,70},{10,90}})));
  Utilities.SinCosEvaluation sinCosEvaluation(phi0=phi0)
    annotation (Placement(transformation(extent={{50,70},{70,90}})));
  Sensors.IncrementalEncoder incrementalEncoder(pRev=pRev)
    annotation (Placement(transformation(extent={{30,40},{10,60}})));
  Utilities.EncoderTimeAndCount encoderTimeAndCount(pRev=pRev, phi0=phi0)
    annotation (Placement(transformation(extent={{50,40},{70,60}})));
  Utilities.EncoderTimeSpan encoderTimeSpan(pRev=pRev, phi0=phi0)
    annotation (Placement(transformation(extent={{50,10},{70,30}})));
  Utilities.EncoderPulseCount encoderPulseCount(pRev=pRev, phi0=phi0)
    annotation (Placement(transformation(extent={{50,-20},{70,0}})));
  Sensors.HallSensor hallSensor(p=p,
    m=m,                             phi0=phi0)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={20,-80})));
  ToMSL.UnwrapAngle unwrapAngle(phi0=phi0)
    annotation (Placement(transformation(extent={{50,-60},{70,-40}})));
  Utilities.HallTimeSpan hallTimeSpan(
    p=p,
    m=m,
    phi0=phi0) annotation (Placement(transformation(extent={{50,-90},{70,-70}})));
equation
  connect(position.flange, sinCosResolver.flange)
    annotation (Line(points={{-10,0},{0,0},{0,80},{10,80}}, color={0,0,0}));
  connect(f2pos.y, position.phi_ref)
    annotation (Line(points={{-39,0},{-32,0}}, color={0,0,127}));
  connect(position.flange, incrementalEncoder.flange)
    annotation (Line(points={{-10,0},{0,0},{0,50},{10,50}},   color={0,0,0}));
  connect(incrementalEncoder.y, encoderTimeSpan.u)
    annotation (Line(points={{31,50},{40,50},{40,20},{48,20}},
                                             color={255,0,255}));
  connect(sinCosResolver.y, sinCosEvaluation.u)
    annotation (Line(points={{31,80},{48,80}}, color={0,0,127}));
  connect(position.flange, hallSensor.flange)
    annotation (Line(points={{-10,0},{0,0},{0,-80},{10,-80}},
                                                           color={0,0,0}));
  connect(incrementalEncoder.y, encoderPulseCount.u) annotation (Line(points={{31,50},
          {40,50},{40,-10},{48,-10}},       color={255,0,255}));
  connect(hallSensor.y, unwrapAngle.u)
    annotation (Line(points={{31,-74},{40,-74},{40,-50},{48,-50}},
                                               color={0,0,127}));
  connect(position.flange, angleSensor.flange)
    annotation (Line(points={{-10,0},{0,0},{0,80},{-20,80}},color={0,0,0}));
  connect(position.flange, speedSensor.flange)
    annotation (Line(points={{-10,0},{0,0},{0,60},{-20,60}},color={0,0,0}));
  connect(refFrequency.y, f2pos.u)
    annotation (Line(points={{-69,0},{-62,0}}, color={0,0,127}));
  connect(hallSensor.yC, hallTimeSpan.uC)
    annotation (Line(points={{31,-80},{48,-80}}, color={255,0,255}));
  connect(incrementalEncoder.y, encoderTimeAndCount.u)
    annotation (Line(points={{31,50},{48,50}}, color={255,0,255}));
  annotation (experiment(
      StopTime=3,
      Interval=1e-05,
      Tolerance=1e-06), Documentation(info="<html>
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
    the outputs are interpreted by the blocks <a href=\"modelica://BLDC.Utilities.EncoderTimeSpan\">EncoderTimeSpan</a> 
    and <a href=\"modelica://BLDC.Utilities.EncoderPulseCount\">EncoderPulseCount</a>.</li>
</ul>
<p>
For <code>p=1</code>, all results are identical. 
For <code>p&gt;1</code>, the results of the HallSensor and the SinCosEvaluation are p times the other results. 
</p>
</html>"));
end DemoEncoder;
