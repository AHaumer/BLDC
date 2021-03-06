within BLDC.Utilities;
block EncoderPulseCount "Count pulses of an incremental encoder"
  extends Modelica.Blocks.Interfaces.DiscreteBlock(samplePeriod=0.01);
  extends BLDC.Interfaces.EncoderEvaluation;
  import Modelica.Constants.pi;
  import BLDC.Functions.directionOfRotation;
protected
  Integer count(start=0, fixed=true) "count of edges";
algorithm
  when {edge(A), edge(notA), edge(B), edge(notB)} then
    count:=pre(count) + directionOfRotation(A, pre(A), B, pre(B));
  end when;
  when sampleTrigger then
    w:=pre(count)*2*pi/(4*pRev)/samplePeriod;
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
          rotation=360),
        Line(
          points={{-60,60},{-60,40}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{-20,60},{-20,40}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{20,60},{20,40}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{60,60},{60,40}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{-40,60},{-40,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{0,60},{0,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{40,60},{40,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash)}));
end EncoderPulseCount;
