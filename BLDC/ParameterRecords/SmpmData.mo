within BLDC.ParameterRecords;
record SmpmData
  "Common parameters for synchronous machines with permanent magnet"
  import Modelica.Units.SI;
  import Modelica.Constants.pi;
  import Modelica.Units.Conversions.from_degC;
  parameter String MachineType="mod.Std.";
  parameter Integer mSystems=Modelica.Electrical.Polyphase.Functions.numberOfSymmetricBaseSystems(m)
    "Number of symmetric base systems" annotation(Dialog(enable=false));
  parameter Integer mBasic=integer(m/mSystems) "Number of phases of basic systems" annotation(Dialog(enable=false));
  parameter SI.Torque tauNominal=191 "Nominal torque"
    annotation(Dialog(tab="Nominal operation"));
  parameter SI.AngularVelocity wNominal(displayUnit="rpm")=2*pi*fsNominal/p "Nominal speed"
    annotation(Dialog(tab="Nominal operation"));
  parameter SI.Current IsNominal=100 "Nominal current per phase"
    annotation(Dialog(tab="Nominal operation"));
  parameter SI.Voltage VsNominal=110.5 "Nominal voltage per phase"
    annotation(Dialog(tab="Nominal operation"));
  parameter SI.Temperature TsNominal(displayUnit="degC")=from_degC(20) "Nominal stator temperature"
    annotation(Dialog(tab="Nominal operation"));
  parameter SI.Temperature TrNominal(displayUnit="degC")=from_degC(20) "Nominal damper cage temperature"
    annotation(Dialog(tab="Nominal operation", enable=useDamperCage));
  extends BLDC.ToMSL.SM_PermanentMagnetData(
    Jr=0.29,
    p=2,
    fsNominal=50,
    VsOpenCircuit=100,
    Rs=m/3*0.03,
    TsRef=from_degC(20),
    alpha20s=0,
    Lssigma=m/3*0.1/(2*pi*fsNominal),
    Lmd=m/3*0.3/(2*pi*fsNominal),
    Lmq=Lmd,
    useDamperCage=false,
    Lrsigmad=m/3*0.05/(2*pi*fsNominal),
    Lrsigmaq=Lrsigmad,
    Rrd=m/3*0.04,
    Rrq=Rrd,
    TrRef=from_degC(20),
    alpha20r=0,
    frictionParameters(
      PRef=0,
      wRef=wNominal,
      power_w=2),
    statorCoreParameters(
      PRef=0,
      VRef=VsOpenCircuit,
      wRef=wNominal),
    strayLoadParameters(
      PRef=0,
      IRef=IsNominal,
      wRef=wNominal,
      power_w=1),
    permanentMagnetLossParameters(
      PRef=0,
      c=0,
      IRef=IsNominal,
      power_I=2,
      wRef=wNominal,
      power_w=1));
  annotation (
    defaultComponentName="smpmData",
    defaultComponentPrefixes="parameter",
    Documentation(info="<html>
<p>Basic parameters of synchronous machines with permanent magnet are predefined with default values.</p>
</html>"),
    Icon(graphics={Text(
          extent={{-100,0},{100,-40}},
          textColor={28,108,200},
          textString="%MachineType")}));
end SmpmData;
