function [] = makeTemp(varName, var)

    if nargin < 2
        var = 'F';
    end
    
    switch(var)
        case 'L'
            copyfile('C:\Users\Viet Do\OneDrive\Documents\MatlabCode\MatlabNavigationLibrary\MatlabTemplate\vietMatlabTemplateLib.m', varName);
            edit(varName);
        case 'S'
            copyfile('C:\Users\Viet Do\OneDrive\Documents\MatlabCode\MatlabNavigationLibrary\MatlabTemplate\vietMatlabTemplateScript.m', varName);
            edit(varName);
        case 'F'
            copyfile('C:\Users\Viet Do\OneDrive\Documents\MatlabCode\MatlabNavigationLibrary\MatlabTemplate\vietMatlabTemplate.m', varName);
            edit(varName);
    end
    
end