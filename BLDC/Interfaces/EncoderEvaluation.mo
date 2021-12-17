within BLDC.Interfaces;
partial block EncoderEvaluation "Partial evaluation of an incremental encoder"
  parameter Integer pRev(final min=1, start=128) "Pulses per revolution";
  parameter Modelica.Units.SI.Angle phi0=0 "Initial mechanical angle (zero position)";
  Modelica.Blocks.Interfaces.BooleanInput u[3] "Encoder signals"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput w(final unit="rad/s", displayUnit="rpm", start=0, fixed=true)
    "Mechanical angular velocity"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput phi(final unit="rad", displayUnit="deg")
    "Mechanical angle"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
protected
  Boolean A=u[2];
  Boolean B=u[3];
  Boolean notA=not A;
  Boolean notB=not B;
initial equation
  pre(A)=true;
  pre(B)=false;
  pre(notA)=false;
  pre(notB)=true;
  phi=phi0;
equation
  der(phi)= w;
  annotation (Documentation(info="<html>
<p>Evaluates the input signals from an <a href=\"modelica://BLDC.Sensors.IncrementalEncoder\">IncrementalEncoder</a>:</p>
<ul>
<li>0-track <code>y[1]</code> = 1 pulse per revolution with a length of <code>2*pi/(4*pRev)</code></li>
<li>A-track <code>y[2]</code> = <code>pRev</code> pulses per revolution</li>
<li>B-track <code>y[3]</code> = <code>pRev</code> pulses per revolution, displaced by <code>2*pi/(4*pRev)</code></li>
</ul>
<p>
and determines mechanical angular velocity <code>w</code> and mechanical angle <code>phi</code>:
</p>
<ul>
<li>w    = angular velocity</li>
<li>phi  = angle = integral of w</li>
</ul>
</html>"), Icon(graphics={
        Line(points={{-80,20},{-60,20},{-60,40},{-20,40},{-20,20},{20,20},{20,40},
              {60,40},{60,20},{80,20}}, color={0,0,0}),
        Line(points={{-80,-40},{-40,-40},{-40,-20},{0,-20},{0,-40},{40,-40},{40,-20},
              {80,-20},{80,-40}},      color={0,0,0}),
        Text(
          extent={{60,20},{100,-20}},
          textColor={0,0,0},
          textString="w")}));
end EncoderEvaluation;
