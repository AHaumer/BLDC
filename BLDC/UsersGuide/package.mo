within BLDC;
package UsersGuide "User's Guide"
  extends Modelica.Icons.Information;


annotation (DocumentationClass=true, Documentation(info="<html>
<p>
This library is developed at <a href=\"https://www.oth-regensburg.de/\">OTH Regensburg</a> (Technical University of Applied Sciences), 
Faculty of Electrical Engineering and Information Technology, Prof. Anton Haumer.
</p>
<p>
The goal is to implement models for brushless DC machines (BLDC), 
ie. permanent magnet synchronous machines under the restriction of spatial sinusoidal magnetic field distribution 
and block commuation derived from the signals of a Hall sensor. 
</p>
<h4>Note:</h4>
<p>
Due to the electronic commutator, i.e. determining the active sector, the models are restricted to <code>m=3</code> phases.
</p>
</html>"));
end UsersGuide;
