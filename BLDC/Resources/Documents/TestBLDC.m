eps=1e-3;
orientation=symmetricOrientation(m);
for ko=1:m
    if orientation(ko)<0
        orientation(ko)=orientation(ko)+2*pi;
    end;
end;
for kr=1:2*m
    rotorPosition(kr)=(kr-1)*2*pi/(2*m);
end;
for ko=1:m
	for kr=1:2*m
    	diff(ko,kr)=mod(orientation(ko)-rotorPosition(kr), 2*pi);
        if diff(ko,kr)>pi
        	diff(ko,kr)=diff(ko,kr)-2*pi;
        end;
        if abs(abs(diff(ko,kr))-pi)<eps
        	diff(ko,kr)=0;
        end;
    end;
end;
for kr=1:2*m
	I(kr)=0;
    for ko=1:m
        active(ko,kr)=0;
        if abs(diff(ko,kr)-pi/2)<=2*pi/(2*m)+eps
        	I(kr)=I(kr)+exp(j*diff(ko,kr));
            active(ko,kr)=+1;
        elseif abs(diff(ko,kr)+pi/2)<=2*pi/(2*m)+eps
        	I(kr)=I(kr)-exp(j*diff(ko,kr));
            active(ko,kr)=-1;
        end;
    end;
end;
    
    function y=symmetricOrientation(m)
        if mod(m,2)==0
            if m==2
                y(1)=0;
                y(2)=pi/2;
            else
                y(1:m/2)  =symmetricOrientation(m/2);
                y(m/2+1:m)=y(1:m/2)-pi/m;
            end;
        else
            for k=1:m
                y(k)=(k-1)*2*pi/m;
            end;
        end;
    end