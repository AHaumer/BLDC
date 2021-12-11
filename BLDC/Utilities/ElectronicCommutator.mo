within BLDC.Utilities;
block ElectronicCommutator "Polyphase electronic commutator"
  extends BLDC.BaseBlocks.BaseElectronicCommutator;
  import Modelica.Units.SI;
  import Modelica.ComplexMath.j;
  import Modelica.ComplexMath.arg;
  import Modelica.Math.wrapAngle;
protected
  parameter SI.Angle wrappedOrientation[m]=wrapAngle(orientation, true);
  SI.Angle rotorPosition=wrapAngle(arg(Modelica.ComplexMath.sum({if uC[k] then Modelica.ComplexMath.exp(j*orientation[k]) else Complex(0) for k in 1:m})), true);
equation
  fire_p=fill(false, m);
  fire_n=fill(false, m);
  annotation (Documentation(info="<html>
<p>
For every Hall signal <code>uC[k]=true</code>, the phasor <code>exp(-j*orientation[k])</code> is added, else zero. 
Taking the argument (angle) of the resulting phasor, the rotor position with an uncertainty of <code>&#177;&pi;/(2*m)</code> is determined.
</p>
<p>
The fire signals are determined by comparing the rotor position is compared with the orientation of phases. 
Phase(s) aligned with the rotor position (positive or negative) are set inactive (open). 
Phases ahead of the rotor position are connected with <code>pwm</code>,
phases behind of the rotor position are connected with <code>not pwm</code>. 
</p>
<p>
The signal <code>pwm</code> is either determined by the input, or set to the parameter <code>ConstantPWM</code> (determining the direction).
</p>
</html>"), Icon(graphics={
        Line(points={{-60,30},{-60,50},{0,50},{0,30},{60,30},{60,50}}, color={255,
              0,255}),
        Line(points={{-60,-10},{-20,-10},{-20,10},{40,10},{40,-10},{60,-10}},
            color={255,0,255}),
        Line(points={{-60,-30},{-40,-30},{-40,-50},{20,-50},{20,-30},{60,-30}},
            color={255,0,255})}));
end ElectronicCommutator;
