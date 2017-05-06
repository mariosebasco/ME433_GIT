clear
clc

s = serial('/dev/cu.usbmodem1421','BaudRate',9600,'Timeout',5);
fopen(s);
display('Connected to the PIC')


%Send 'r' to start receiving data
AZ = zeros(100,4);
fprintf(s,'%c','r');

display('sent r')

for i = 1:100      
   AZ(i,:) = str2num(fscanf(s,'%s'));
end

fclose(s);
display('data received')

%%
X = linspace(1,100,100);

figure
plot(X,AZ(:,1))
title('raw data')
axis([0 100 -105 -95])

figure
plot(X,AZ(:,2))
title('MAF')
axis([0 100 -105 -95])

figure
plot(X,AZ(:,3))
title('FIR')
axis([0 100 -105 -95])

figure
plot(X,AZ(:,4))
title('IIR')
axis([0 100 -105 -95])


