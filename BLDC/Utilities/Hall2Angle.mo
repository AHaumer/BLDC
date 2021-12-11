within BLDC.Utilities;
block Hall2Angle "Detects rotor angle from hall signals"
  extends Modelica.Blocks.Interfaces.SO(y(unit="rad", displayUnit="deg"));
  import Modelica.Units.SI;
  import Modelica.ComplexMath.j;
  import Modelica.ComplexMath.arg;
  import Modelica.Math.wrapAngle;
  parameter Integer m(min=3) = 3 "Number of stator phases";
  parameter SI.Angle orientation[m]=
    Modelica.Electrical.Polyphase.Functions.symmetricOrientation(m) "Orientation of phases";
  Modelica.Blocks.Interfaces.BooleanInput uC[m] "Commutation signals"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=0,  origin={-120,0})));
protected
  SI.Angle rotorPosition=arg(Modelica.ComplexMath.sum({if uC[k] then Modelica.ComplexMath.exp(j*orientation[k]) else Complex(0) for k in 1:m}));
equation
  y=rotorPosition;
  annotation (Documentation(info="<html>
<p>
For every Hall signal <code>uC[k]=true</code>, the phasor <code>exp(-j*orientation[k])</code> is added, else zero. 
Taking the argument (angle) of the resulting phasor, the rotor angle with an uncertainty of <code>&#177;&pi;/(2*m)</code> is determined.
</p>
</html>"), Icon(graphics={
        Line(points={{-60,30},{-60,50},{0,50},{0,30},{60,30},{60,50}}, color={255,
              0,255}),
        Line(points={{-60,-10},{-20,-10},{-20,10},{40,10},{40,-10},{60,-10}},
            color={255,0,255}),
        Line(points={{-60,-30},{-40,-30},{-40,-50},{20,-50},{20,-30},{60,-30}},
            color={255,0,255})}));
end Hall2Angle;
