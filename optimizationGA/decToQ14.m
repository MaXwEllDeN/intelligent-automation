function val = decToQ14(num, hexSize)
    N0 = round(2^(14) * num);
    pattern = strcat("%", sprintf("0%dx", hexSize));

    if sum(size(num)) == 2
        val = sprintf(pattern, N0);
    else
        val = char(zeros(size(N0, 1), hexSize));

        for i = 1:length(num)
            val(i, :) = char(sprintf(pattern, N0(i)));
        end
    end
end
