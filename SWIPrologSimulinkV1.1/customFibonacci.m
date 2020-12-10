function f = customFibonacci(n)
if n <= 1
    f = 1;
else
    f = customFibonacci(n-1) + customFibonacci(n-2);
end