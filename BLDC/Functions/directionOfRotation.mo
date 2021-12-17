within BLDC.Functions;
function directionOfRotation "Detect direction of rotation"
  extends Modelica.Icons.Function;
  input Boolean A "Track A";
  input Boolean preA "pre(Track A)";
  input Boolean B "Track B";
  input Boolean preB "pre(Track B)";
  output Integer dir;
algorithm
  dir:= if ((A and not preA and not B) or (not A and preA and B) or
            (B and not preB and A) or (not B and preB and not A)) then +1
    elseif ((A and not preA and B) or (not A and preA and not B) or
            (B and not preB and not A) or (not B and preB and A)) then -1
    else 0;
annotation(Inline=true);
end directionOfRotation;
