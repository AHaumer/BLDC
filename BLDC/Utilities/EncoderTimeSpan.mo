within BLDC.Utilities;
block EncoderTimeSpan "Time span between edges of incremental encoder signals"
  extends Modelica.Blocks.Icons.DiscreteBlock;
  import Modelica.Constants.pi;
  import Modelica.Constants.eps;
  parameter Integer pRev(final min=1, start=128) "Pulses per revolution";
  parameter Modelica.Units.SI.Angle phi0=0 "Initial mechanical angle (zero position)";
  Modelica.Blocks.Interfaces.BooleanInput u[3] "Encoder signals"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput w(final unit="rad/s", displayUnit="rpm")
    "Mechanical anglar velocity"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput phi(final unit="rad", displayUnit="deg")
    "Mechanical angle"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
protected
  Boolean A=u[2];
  Boolean B=u[3];
  Boolean notA=not A;
  Boolean notB=not B;
  Boolean firstEdge;
  Modelica.Units.SI.Time t0;
initial equation
  pre(A)=true;
  pre(B)=false;
  pre(notA)=false;
  pre(notB)=true;
  firstEdge=true;
  t0=time;
  phi=phi0;
equation
  der(phi)= w;
  when edge(A) then
    w=(if notB then 1 else -1)*(if noEvent(time - pre(t0) < eps) or pre(firstEdge) then 0 else 2*pi/(4*pRev)/(time - pre(t0)));
    firstEdge=false;
    t0=time;
  elsewhen edge(notA) then
    w=(if B then 1 else -1)*(if noEvent(time - pre(t0) < eps) or pre(firstEdge) then 0 else 2*pi/(4*pRev)/(time - pre(t0)));
    firstEdge=false;
    t0= time;
  elsewhen edge(B) then
    w=(if A then 1 else -1)*(if noEvent(time - pre(t0) < eps) or pre(firstEdge) then 0 else 2*pi/(4*pRev)/(time - pre(t0)));
    firstEdge=false;
    t0= time;
  elsewhen edge(notB) then
    w=(if notA then 1 else -1)*(if noEvent(time - pre(t0) < eps) or pre(firstEdge) then 0 else 2*pi/(4*pRev)/(time - pre(t0)));
    firstEdge=false;
    t0= time;
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
The time span from one edge of the input signals to the next is measured, thus calculating the speed of rotation: <code>2*pi/(4*pRev)/(time - t0)</code>.
Taking both rising and falling edges of A-track and B-track into account, <code>4*pRev</code> edges per revolution are observed.
Note that the output is discrete w.r.t. to time, it is not differentiable.
The algorithm depends on interrupts (which appear in Modelica as when-clauses). 
The direction of rotation has to be detected by a logic from the values of A-track and B-track. 
Note that this algorithm depends on the accuracy of the representation of time, 
but usually gives better results than counting edges during a time gate.
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
          points={{-60,80},{-60,40}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{-40,80},{-40,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(points={{-80,70},{-20,70}},color={0,0,0}),
        Polygon(
          points={{3,0},{-11,6},{-11,-6},{3,0}},
          lineColor={0,0,0},
          fillColor={235,235,235},
          fillPattern=FillPattern.Solid,
          origin={-63,70},
          rotation=360),
        Polygon(
          points={{3,0},{-11,6},{-11,-6},{3,0}},
          lineColor={0,0,0},
          fillColor={235,235,235},
          fillPattern=FillPattern.Solid,
          origin={-37,70},
          rotation=180)}));
end EncoderTimeSpan;
