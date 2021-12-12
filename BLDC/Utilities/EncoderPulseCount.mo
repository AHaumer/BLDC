within BLDC.Utilities;
block EncoderPulseCount "Count pulses of an incremental encoder"
  extends Modelica.Blocks.Interfaces.DiscreteBlock(samplePeriod=0.01);
  import Modelica.Constants.pi;
  parameter Integer pRev(final min=1, start=128) "Pulses per revolution";
  parameter Modelica.Units.SI.Angle phi0=0 "Initial mechanical angle (zero position)";
  Modelica.Blocks.Interfaces.BooleanInput u[3] "Encoder signals"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput w(final unit="rad/s", displayUnit="rpm")
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
  Integer count "count of edges";
initial equation
  pre(A)=true;
  pre(B)=false;
  pre(notA)=false;
  pre(notB)=true;
  count=0;
  phi=phi0;
equation
  der(phi)= w;
algorithm
  when edge(A) then
    count:=pre(count) + (if notB then 1 else -1);
  elsewhen edge(notA) then
    count:=pre(count) + (if B then 1 else -1);
  elsewhen edge(B) then
    count:=pre(count) + (if A then 1 else -1);
  elsewhen edge(notB) then
    count:=pre(count) + (if notA then 1 else -1);
  end when;
  when sampleTrigger then
    w:=count*2*pi/(4*pRev)/samplePeriod;
    count:=0;
  end when;
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
<p>
During the samplePeriod all rising and falling edges of A-track and B-track are counted. 
The direction of rotation has to be detected by a logic from the values of A-track and B-track. 
From that count, the angular velocity can be calculated: <code>w=count*2*pi(4*pRev)/samplePeriod</code>. 
The accuracy of the result depends on the number of pulses per revolution and the samplePeriod. 
Note that the output is discrete w.r.t. to time, it is not differentiable.
The algorithm depends on interrupts (which appear in Modelica as when-clauses).
</p>
</html>"), Icon(graphics={
        Line(points={{-80,20},{-60,20},{-60,40},{-20,40},{-20,20},{20,20},{20,40},
              {60,40},{60,20},{80,20}}, color={0,0,0}),
        Line(points={{-80,-40},{-40,-40},{-40,-20},{0,-20},{0,-40},{40,-40},{40,-20},
              {80,-20},{80,-40}},      color={0,0,0}),
        Text(
          extent={{60,20},{100,-20}},
          textColor={0,0,0},
          textString="w"),
        Line(
          points={{-70,80},{-70,60}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(points={{-70,70},{70,70}}, color={0,0,0}),
        Line(
          points={{70,80},{70,60}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Polygon(
          points={{3,0},{-11,6},{-11,-6},{3,0}},
          lineColor={0,0,0},
          fillColor={235,235,235},
          fillPattern=FillPattern.Solid,
          origin={-67,70},
          rotation=180),
        Polygon(
          points={{3,0},{-11,6},{-11,-6},{3,0}},
          lineColor={0,0,0},
          fillColor={235,235,235},
          fillPattern=FillPattern.Solid,
          origin={67,70},
          rotation=360)}));
end EncoderPulseCount;
