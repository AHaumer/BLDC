within BLDC.Utilities;
block EncoderTimeAndCount
  "Measure time spaen between first and last edge of an incremental encoder with samplePrriod"
  extends Modelica.Blocks.Interfaces.DiscreteBlock(samplePeriod=0.01);
  extends BLDC.Interfaces.EncoderEvaluation;
  import Modelica.Constants.pi;
  import Modelica.Constants.eps;
  import BLDC.Functions.directionOfRotation;
protected
  Integer count(start=0, fixed=true) "count of edges";
  Boolean firstEdge(start=true, fixed=true) "true at first edge";
  discrete Modelica.Units.SI.Time t1(start=0, fixed=true) "time instant of first edge";
  discrete Modelica.Units.SI.Time t2(start=0, fixed=true) "time instant of last edge";
algorithm
  when {edge(A), edge(notA), edge(B), edge(notB)} then
    count:=pre(count) + directionOfRotation(A, pre(A), B, pre(B));
    if pre(firstEdge) then
      firstEdge:=false;
      t1:=time;
    else
      t2:=time;
    end if;
  end when;
  when sampleTrigger then
    w:=if noEvent(pre(t2) - pre(t1) < eps) then 0
  else sign(pre(count))*(abs(pre(count)) - 1)*2*pi/(4*pRev)/(pre(t2) - pre(t1));
    count:=0;
    firstEdge:=true;
    t1:=0;
    t2:=0;
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
From the time span between first and last edge within the samplePeriod and the count of edges, 
the angular velocity can be calculated: <code>w=(count - 1)*2*pi(4*pRev)/(t_last - t_first)</code>. 
Note that the output is discrete w.r.t. to time, it is not differentiable.
The algorithm depends on interrupts (which appear in Modelica as when-clauses).
</p>
</html>"), Icon(graphics={
        Line(
          points={{-60,80},{-60,60}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(points={{-60,70},{40,70}}, color={0,0,0}),
        Line(
          points={{40,80},{40,60}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Polygon(
          points={{3,0},{-11,6},{-11,-6},{3,0}},
          lineColor={0,0,0},
          fillColor={235,235,235},
          fillPattern=FillPattern.Solid,
          origin={-57,70},
          rotation=180),
        Polygon(
          points={{3,0},{-11,6},{-11,-6},{3,0}},
          lineColor={0,0,0},
          fillColor={235,235,235},
          fillPattern=FillPattern.Solid,
          origin={37,70},
          rotation=360),
        Line(
          points={{-60,60},{-60,40}},
          color={0,0,0},
          pattern=LinePattern.Dash),
        Line(
          points={{40,60},{40,-20}},
          color={0,0,0},
          pattern=LinePattern.Dash)}));
end EncoderTimeAndCount;
