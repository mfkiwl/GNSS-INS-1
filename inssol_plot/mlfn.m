function val = mlfn(e0, e1, e2, e3, phi)

    val = e0 * phi - e1 * sin(2.0 * phi) + e2 * sin(4.0 * phi) - e3 * sin(6.0 * phi);

end