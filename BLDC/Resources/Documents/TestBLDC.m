function I=TestBLDC(m)
    orientation=symmetricOrientation(m);
    for k=1:2*m
        rotorPosition(k)=(k-1)*2*pi/(2*m);
    end;
    for ko=1:m
        for kr=1:2*m
            diff(ko,kr)=mod(orientation(ko)-rotorPosition(kr), 2*pi);
            if diff(ko,kr)>pi
                diff(ko,kr)=diff(ko,kr)-2*pi;
            end;
            if abs(diff(ko,kr)-pi)<1e-6
                diff(ko,kr)=0;
            end;
        end;
    end;
    for kr=1:2*m
        I(kr)=0;
        for ko=1:m
            if diff(ko,k)>1e-6
                I(kr)=I(kr)+exp(j*diff(ko,k));
            elseif diff(ko,k)<-1e-6
                I(kr)=I(kr)-exp(j*diff(ko,k));
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
end