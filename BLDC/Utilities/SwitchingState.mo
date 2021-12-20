within BLDC.Utilities;
model SwitchingState "Stationary switching state"
  extends Modelica.Blocks.Icons.Block;
  extends Modelica.Electrical.PowerConverters.Interfaces.ACDC.ACplug;
  extends Modelica.Electrical.PowerConverters.Interfaces.ACDC.DCtwoPin;
  parameter Boolean on[m]={true,false,false} "On-state of phases";
equation
  for k in 1:m loop
    if on[k] then
      connect(ac.pin[k], dc_p);
    else
      connect(ac.pin[k], dc_n);
    end if;
  end for;
end SwitchingState;
