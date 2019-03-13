# SPDX-License-Identifier: CC-BY-SA-4.0
# Copyright 2018-2019 Peter A. Bigot

# Determine the Steinhart-Hart coefficients for a thermistor by solving
# the equations using three data points from the experimental tables in
# the data sheet.

# Offset to add to a Cel temperature to get K
global Cel_Offset = 273.15;

function calcSH (tag, data)
  format short g;
  format compact;
  global Cel_Offset;

  printf("\n%s temperature and resistance datapoints:\n", tag);
  disp(data);

  # Temperature values
  T = data(:, 1);

  # Resistance values
  R = data(:, 2);

  # Result side of the Steinhart-Hart equations (1/T)
  B = 1.0 ./ T;

  # Matrix side of the Steinhart-Hart equations (a + b*ln(R) + c*(ln(R))^3)
  lr = log(R);
  A(:,1) = [ 1, 1, 1];
  A(:,2) = lr;
  A(:,3) = lr .* lr .* lr;

  # Solve the equations
  coeff = A \ B;

  # Top and bottom values in ADC counts.
  # The lower temperature produces a lower ADC measurement closer to an
  # open thermistor.
  # The higher temperature produces a higher ADC measurement closer to a
  # shorted thermistor.
  # Calculations below assume the thermistor is the upper resistor
  # (connected directly to Vdd).
  Ref_Ohm = 10000;
  open_adc16 = floor(Ref_Ohm * 2^16 / (R(1) + Ref_Ohm));
  open_Ohm = Ref_Ohm * (2^16 - open_adc16) / open_adc16;
  lr = log(open_Ohm);
  open_Cel = 1 / (coeff(1) + lr * (coeff(2) + lr * lr * coeff(3))) - Cel_Offset;
  shorted_adc16 = ceil(Ref_Ohm * 2^16 / (R(3) + Ref_Ohm));
  shorted_Ohm = Ref_Ohm * (2^16 - shorted_adc16) / shorted_adc16;
  lr = log(shorted_Ohm);
  shorted_Cel = 1 / (coeff(1) + lr * (coeff(2) + lr * lr * coeff(3))) - Cel_Offset;

  # Display the results
  disp("Steinhart-Hart coefficients:");
  printf(".a = %.17e,\n.b = %.17e,\n.c = %.17e,\n", coeff(1), coeff(2), coeff(3));
  printf(".open_adc16 = %u,\t// %.2f Cel\n.shorted_adc16 = %u,\t// %.2f Cel\n",
         open_adc16, open_Cel,
         shorted_adc16, shorted_Cel);

  deltas = B' - (A * coeff)';
  disp("Check deltas:"); disp(deltas);
endfunction

# https://www.adafruit.com/product/372 supports -40 to 200 Cel
# NB: PVC on wires limited to 105 Cel
# Table: http://www.adafruit.com/datasheets/103_3950_lookuptable.pdf

# Full-scale: [-40, 200] Cel
calcSH("FS", [
              -40 + Cel_Offset , 277200 ;
               25 + Cel_Offset ,  10000 ;
              200 + Cel_Offset ,     61.9 ;
]);

# Wide-range HVAC: [-40, 75] Cel or [-40, 167] [Fahr]
calcSH("HVAC", [
                -40 + Cel_Offset , 277200 ;
                 20 + Cel_Offset ,  12470 ;
                 75 + Cel_Offset ,   1465 ;
]);

# Narrow-Range Cool: [-30, 10] Cel or [-22, 50] [Fahr] for refrigerator
calcSH("Refrigerator", [
                        -30 + Cel_Offset , 157200 ;
                          0 + Cel_Offset ,  31770 ;
                         10 + Cel_Offset ,  19680 ;
]);

# Local Variables:
# indent-tabs-mode: nil
# End:
