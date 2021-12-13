within BLDC.Utilities;
block SinCosEvaluation "Evaluation of Sin/Cos-resolver signals"
  extends Modelica.Blocks.Icons.Block;
  parameter Modelica.Units.SI.Time Ti=1e-6 "Integral time constant of controller";
  parameter Modelica.Units.SI.Angle phi0=0 "Initial mechanical angle (zero position)";
  Modelica.Blocks.Interfaces.RealInput u[4] "Resolver signals"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput phi(final unit="rad", displayUnit="deg")
    "Electrical angle"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput w(final unit="rad/s", displayUnit="rpm")
    "Electrical angular velocity"
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
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
to obtain the electrical angle <code>phi</code> and the electrial angular velocity <code>w</code>:
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
</html>"), Icon(graphics={
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
