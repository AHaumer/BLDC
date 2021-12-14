within BLDC.Utilities;
block HallTimeSpan "Time span between edges of Hall signals"
  extends Modelica.Blocks.Icons.DiscreteBlock;
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  import Modelica.Constants.eps;
  import BLDC.Utilities.Functions.getEventIndex;
  import BLDC.Utilities.Functions.addIndex;
  parameter Integer p(final min=1, start=2) "Number of pole pairs";
  parameter Integer m(min=3) = 3 "Number of stator phases";
  parameter SI.Angle orientation[m]=
    Modelica.Electrical.Polyphase.Functions.symmetricOrientation(m) "Orientation of phases";
  parameter SI.Angle phi0=0 "Initial mechanical angle (zero position)";
  Modelica.Blocks.Interfaces.BooleanInput uC[m] "Hall signals"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput w(final unit="rad/s", displayUnit="rpm")
    "Mechanical anglar velocity"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput phi(final unit="rad", displayUnit="deg")
    "Mechanical angle"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
protected
  Boolean notC[m]=not uC;
  Boolean firstEdge "true at first edge";
  Modelica.Units.SI.Time t0 "time instant of last edge";
initial equation
  pre(uC)=fill(false,m);
  pre(notC)=fill(true,m);
  firstEdge=true;
  t0=time;
  phi=phi0;
equation
  der(phi)= w;
  when edge(uC) then
    w= (if not uC[addIndex(getEventIndex(m,uC,pre(uC)), 1, m)] then +1 else -1)*
       (if noEvent(time - pre(t0) < eps) or pre(firstEdge) then 0 else 2*pi/(2*m)/(time - pre(t0)));
    firstEdge= false;
    t0= time;
  elsewhen edge(notC) then
    w= (if not uC[addIndex(getEventIndex(m,notC,pre(notC)), 1, m)] then -1 else +1)*
       (if noEvent(time - pre(t0) < eps) or pre(firstEdge) then 0 else 2*pi/(2*m)/(time - pre(t0)));
    firstEdge= false;
    t0= time;
  end when;
  annotation (Documentation(info="<html>
<p>
Evaluates the input signals from a <a href=\"modelica://BLDC.Sensors.HallSensor\">HallSensor</a>: a 180&deg;-rectangle per phase (per pole pair) 
and determines electrical angular velocity <code>w</code> and electrical angle <code>phi</code>:
</p>
<ul>
<li>w    = angular velocity</li>
<li>phi  = angle = integral of w</li>
</ul>
<p>
The time span from one edge of the input signals to the next is measured, thus calculating the speed of rotation: <code>2*pi/(p*2*m)/(time - t0)</code>.
Taking both rising and falling edges into account, <code>p*2*m</code> edges per revolution are observed.
Note that the output is discrete w.r.t. to time, it is not differentiable.
The algorithm depends on interrupts (which appear in Modelica as when-clauses). 
The direction of rotation has to be detected by a logic from the sequence of the Hall signals. 
Note that this algorithm depends on the accuracy of the representation of time, 
but usually gives better results than counting edges during a time gate.
</p>
</html>"), Icon(graphics={
        Text(
          extent={{60,20},{100,-20}},
          textColor={0,0,0},
          textString="w"),
        Line(
          points={{-60,80},{-60,50}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{-40,80},{-40,-30}},
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
          rotation=180),
        Line(points={{-60,30},{-60,50},{0,50},{0,30},{60,30},{60,50}}, color={255,
              0,255}),
        Line(points={{-60,-10},{-20,-10},{-20,10},{40,10},{40,-10},{60,-10}},
            color={255,0,255}),
        Line(points={{-60,-30},{-40,-30},{-40,-50},{20,-50},{20,-30},{60,-30}},
            color={255,0,255})}));
end HallTimeSpan;
