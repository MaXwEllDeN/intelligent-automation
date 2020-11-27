function val = q14ToDec(hex)
    N0 = hex2dec(hex);
    val = N0 / (2^14);
end