
function [x,y] = Trans_TM(lat, lon)
% S = shaperead('contour_tm.shp');
% % data = load('NMPM21060038E_2021-04-23_05-02-47.ASC'); % 커버 x
% % data = load('NMPM21060038E_2021-04-23_06-07-27.ASC'); % 커버 0
% data = load('NMPM21060038E_2021-04-23_06-39-50.ASC'); % 커버 0

d2r = pi / 180.0;

% lat = data(:,4).*d2r;    % rad
% lon = data(:,5).*d2r;    % rad
lat = lat.*d2r;    % rad
lon = lon.*d2r;    % rad

for i=1:length(lat)
    % // Set Ellips factor
    m_arMajor = 6378137.0;
    m_arMinor = 6356752.3142;
    
    %// Set System Factor
    m_arScaleFactor = 1;
    
    %     중부 원점
    m_arLonCenter = 127.0 * d2r;
    m_arLatCenter = 38.0 * d2r;
    m_arFalseNorthing = 600000;%// 1524.0); to X axis (+)
    m_arFalseEasting = 200000;%// -1760.0); to Y axis (-)
    
    %// Set Internal Values
    temp = m_arMinor / m_arMajor;
    m_dDstEs = 1.0 - temp * temp;
    m_dDstE = sqrt(m_dDstEs);
    m_dDstE0 = e0fn(m_dDstEs);
    m_dDstE1 = e1fn(m_dDstEs);
    m_dDstE2 = e2fn(m_dDstEs);
    m_dDstE3 = e3fn(m_dDstEs);
    m_dDstMl0 = m_arMajor * mlfn(m_dDstE0, m_dDstE1, m_dDstE2, m_dDstE3, m_arLatCenter);
    m_dDstEsp = m_dDstEs / (1.0 - m_dDstEs);
    
    % 	double m_dDstInd;
    if (m_dDstEs < 0.00001)
        m_dDstInd = 1.0;
    else
        m_dDstInd = 0.0;
        
        % 	double delta_lon; %// Delta longitude (Given longitude - center longitude)
        % 	double sin_phi, cos_phi; %// sin and cos value
        % 	double al, als; %// temporary values
        % 	double b, c, t, tq; %// temporary values
        % 	double con, n, ml; %// cone constant, small m
        
        % 	// LL to TM Forward equations from here
        delta_lon = lon(i) - m_arLonCenter;
        sin_phi = sin(lat(i));
        cos_phi = cos(lat(i));
        
        if (m_dDstInd ~= 0)
            
            b = cos_phi * sin(delta_lon);
            % 		if ((fabs(fabs(b) - 1.0)) < 0.0000000001) {;}
            
        else
            
            b = 0;
            x(i) = 0.5 * m_arMajor * m_arScaleFactor * log((1.0 + b) / (1.0 - b));
            con = acos(cos_phi * cos(delta_lon) / sqrt(1.0 - b * b));
            if (lat(i) < 0)
                
                con = -con;
                y(i) = m_arMajor * m_arScaleFactor * (con - m_arLatCenter);
            end
        end
    end
    
    al = cos_phi * delta_lon;
    als = al * al;
    c = m_dDstEsp * cos_phi * cos_phi;
    tq = tan(lat(i));
    t = tq * tq;
    con = 1.0 - m_dDstEs * sin_phi * sin_phi;
    n = m_arMajor / sqrt(con);
    ml = m_arMajor * mlfn(m_dDstE0, m_dDstE1, m_dDstE2, m_dDstE3, lat(i));
    
    y(i) = m_arScaleFactor * n * al * (1.0 + als / 6.0 * (1.0 - t + c + als / 20.0 * (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * m_dDstEsp))) + m_arFalseEasting;
    x(i) = m_arScaleFactor * (ml - m_dDstMl0 + n * tq * (als * (0.5 + als / 24.0 * (5.0 - t + 9.0 * c + 4.0 * c * c + als / 30.0 * (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * m_dDstEsp))))) + m_arFalseNorthing;
end
% orgx = S(1).X(1,1);
% orgy = S(1).Y(1,1);
% figure,
% for i=1:length(S)
%     plot(S(i).X(1,:)-orgx,S(i).Y(1,:)-orgy,'k.-')
%     hold on
% end

% plot(y-orgx,x-orgy,'.')
end