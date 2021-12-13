within BLDC.ToMSL;
block UnwrapAngle "Angle tracking observer"
  extends Modelica.Blocks.Interfaces.SISO(u(final unit="rad", displayUnit="deg"),
    y(final unit="rad", displayUnit="deg"));
  parameter Modelica.Units.SI.Time Ti=1e-6 "Integral time constant of controller";
  parameter Modelica.Units.SI.Angle phi0=0 "Initial angle";
  Modelica.Blocks.Interfaces.RealOutput w(final unit="rad/s", displayUnit="rpm")
    "Angular velocity"
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
  Modelica.Blocks.Math.Cos cos1
    annotation (Placement(transformation(extent={{-70,-30},{-50,-10}})));
  Modelica.Blocks.Math.Sin sin1
    annotation (Placement(transformation(extent={{-70,10},{-50,30}})));
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
  connect(u, sin1.u) annotation (Line(points={{-120,0},{-80,0},{-80,20},{-72,20}},
        color={0,0,127}));
  connect(u, cos1.u) annotation (Line(points={{-120,0},{-80,0},{-80,-20},{-72,-20}},
        color={0,0,127}));
  connect(integralController.y,rotator. angle) annotation (Line(points={{61,0},{
          80,0},{80,20},{-20,20},{-20,12}}, color={0,0,127}));
  connect(gain.y,integralController. u)
    annotation (Line(points={{21,0},{38,0}}, color={0,0,127}));
  connect(rotator.y[2],gain. u)
    annotation (Line(points={{-9,0},{-2,0}}, color={0,0,127}));
  connect(sin1.y, rotator.u[2]) annotation (Line(points={{-49,20},{-40,20},{-40,
          0},{-32,0}}, color={0,0,127}));
  connect(cos1.y, rotator.u[1]) annotation (Line(points={{-49,-20},{-40,-20},{-40,
          0},{-32,0}}, color={0,0,127}));
  connect(integralController.y, y)
    annotation (Line(points={{61,0},{110,0}}, color={0,0,127}));
  connect(gain.y, w) annotation (Line(points={{21,0},{30,0},{30,-60},{110,-60}},
        color={0,0,127}));
  annotation (Icon(graphics={
        Text(
          extent={{58,-40},{100,-80}},
          textColor={28,108,200},
          textString="w"),
        Polygon(
          points={{0,90},{-8,68},{8,68},{0,90}},
          lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid),
        Line(points={{0,-80},{0,68}}, color={192,192,192}),
        Polygon(
          points={{90,0},{68,8},{68,-8},{90,0}},
          lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid),
        Line(points={{-90,0},{68,0}}, color={192,192,192}),
        Line(points={{-72,20},{-72,20},{-40,60},{-40,-40},{40,60},{40,-60},{56,-40}}),
        Line(points={{56,80},{-72,-80}}, color={28,108,200},
          thickness=0.5)}),
      Documentation(info="<html>
<p>
An angle tracking observer is a very robust method to determine the angle of a space phasor. 
If we calculate <code>cos</code> and <code>sin</code> of a wrapped angle - no matter whether in the interval [0, 2&pi;) or (-&pi;, +&pi;] - 
we can use this algorithm to unwrap the angle.
</p>
<p>
Rotating the space phasor by an angle that is determined by the controller - whose goal is to bring the imaginary part to zero - the result is the desired continuos angle. 
The result can be differentiated to obtain the angular velocity, but as a bonus the input of the integral controller already is the angular velocity. 
The result approximates the desired angle by a firstOrder whose time constant is the integral time constant:
</p>
<p>
<code>Im(e<sup>j(&phi;-&phi;')</sup>)=sin(&phi;-&phi;')</code> which can be approximated by <code>(&phi;-&phi;')</code>
</p>
<p>
Using an integral contoller, the transfer function of the closed loop can be determined as:
<code>&phi;'=&phi;/(1 + s*T<sub>I</sub>)</code>.
</p>
</html>"));
end UnwrapAngle;
