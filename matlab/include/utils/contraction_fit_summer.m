function sym_poly_fit = contraction_fit_summer(p)
%CONTRACTION_FIT_SUMMER
%    SYM_POLY_FIT = CONTRACTION_FIT_SUMMER(P)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    06-Jul-2022 20:54:17

t2 = conj(p);
et1 = t2.*(-3.982361379193073e-4)-t2.^2.*8.2162426562891e-5+t2.^3.*5.394266367122178e-7+t2.^4.*2.666729951796928e-8;
et2 = t2.^5.*(-3.002668523029962e-10)+4.269760703097809e-1;
sym_poly_fit = et1+et2;
