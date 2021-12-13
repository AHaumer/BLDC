within BLDC.Utilities;
block ElectronicCommutator "Polyphase electronic commutator"
  extends Modelica.Blocks.Icons.BooleanBlock;
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  import Modelica.ComplexMath.j;
  import Modelica.ComplexMath.arg;
  import Modelica.Math.wrapAngle;
  import Modelica.Electrical.Polyphase.Functions.symmetricOrientation;
  parameter Integer m(min=3) = 3 "Number of stator phases";
  parameter Modelica.Units.SI.Angle orientation[m]=wrapAngle(symmetricOrientation(m), true)
    "Orientation of phases within [0, 2*pi)";
  parameter Boolean useConstantPWM=true "Otherwise input" annotation(Evaluate=true);
  parameter Boolean ConstantPWM=true "PWM resp. direction" annotation(Dialog(enable=useConstantPWM));
  Modelica.Blocks.Interfaces.BooleanInput pwm if not useConstantPWM
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.BooleanInput uC[m] "Commutation signals"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Modelica.Blocks.Interfaces.BooleanOutput fire_p[m]
    "Fire signals of positive potential transistors"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.BooleanOutput fire_n[m]
    "Fire signals of negative potential transistors"
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
  output Integer sector "Rotorposition sector";
  output Integer state[m] "Fire state of phases";
protected
  constant SI.Angle eps=1e-3;
  Complex cRotorPosition=Modelica.ComplexMath.sum({if uC[k] then Modelica.ComplexMath.exp(j*orientation[k]) else Complex(0) for k in 1:m})
    "Phasor pointing into the direction of the rotor";
  SI.Angle rotorPosition=wrapAngle(arg(cRotorPosition), true) "Rotor position in the range [0, 2*pi]";
  SI.Angle diff[m] "Difference between orientation and rotor position";
  Modelica.Blocks.Interfaces.BooleanInput internalPWM annotation (Placement(
        transformation(extent={{-94,56},{-86,64}}), iconTransformation(extent={{-94,
            56},{-86,64}})));
algorithm
  diff:=wrapAngle(orientation - fill(rotorPosition, m), false);
  diff:={if abs(abs(diff[k]) - pi) < eps then 0 else diff[k] for k in 1:m};
equation
  if useConstantPWM then
    internalPWM=ConstantPWM;
  else
    connect(internalPWM, pwm);
  end if;
  assert(mod(m,2)<>0, "Electronic commutator not working properly for even number of phases!");
  sector=integer(2*m*rotorPosition/(2*pi));
  for k in 1:m loop
    if     abs(diff[k] - pi/2)<=pi/(2*m)+eps then
      state[k]=+1;
      fire_p[k]=internalPWM;
      fire_n[k]=not internalPWM;
    elseif abs(diff[k] + pi/2)<=pi/(2*m)+eps then
      state[k]=-1;
      fire_p[k]=not internalPWM;
      fire_n[k]=internalPWM;
    else
      state[k]= 0;
      fire_p[k]=false;
      fire_n[k]=false;
    end if;
  end for;
/*
  fire_p={if abs(diff[k] - pi/2)<=pi/(2*m)+eps then internalPWM
      elseif abs(diff[k] + pi/2)<=pi/(2*m)+eps then not internalPWM else false for k in 1:m};
  fire_n={if abs(diff[k] + pi/2)<=pi/(2*m)+eps then internalPWM
      elseif abs(diff[k] - pi/2)<=pi/(2*m)+eps then not internalPWM else false for k in 1:m};
*/
  annotation (Documentation(info="<html>
<p>
For every Hall signal <code>uC[k]=true</code>, the phasor <code>exp(-j*orientation[k])</code> is added, else zero. 
Taking the argument (angle) of the resulting phasor, the rotor position with an uncertainty of <code>&#177;&pi;/(2*m)</code> is determined.
</p>
<p>
The fire signals are determined by comparing the rotor position is compared with the orientation of phases. 
Phase(s) aligned with the rotor position (positive or negative) are set inactive (open). 
Phases ahead +90&deg; of the rotor position are connected with <code>pwm</code>,
phases behind -90&deg; of the rotor position are connected with <code>not pwm</code>. 
</p>
<p>
The signal <code>pwm</code> is either determined by the input, or set to the parameter <code>ConstantPWM</code> (determining the direction).
</p>
<h4>Note:</h4>
<p>
Phases with orientation within a span of <code>&#177;&pi;/(2*m)</code> around <code>rotorPosition&#177;&pi;/2</code> are choosen as active, the remaining as inactive (open).
The electronic commutator is working properly for odd number of phases, even number of phases nees further investigation. 
</p>
</html>"), Icon(graphics={
        Line(points={{-60,30},{-60,50},{0,50},{0,30},{60,30},{60,50}}, color={255,
              0,255}),
        Line(points={{-60,-10},{-20,-10},{-20,10},{40,10},{40,-10},{60,-10}},
            color={255,0,255}),
        Line(points={{-60,-30},{-40,-30},{-40,-50},{20,-50},{20,-30},{60,-30}},
            color={255,0,255}),
        Text(
          extent={{-100,-70},{100,-90}},
          textColor={0,0,0},
          textString="m=%m")}));
end ElectronicCommutator;
