// ============================================
//              NED FILE
//       SDN Network Topology
// ============================================

// =========================
//      SDN Components
// =========================

simple sdn_switch {
    parameters:
        @signal[arrival](type="long");
        @statistic[hopCount](title="hop count"; source="arrival"; record=vector,stats; interpolationmode=none);
        volatile double transmissionDelay;
        volatile double queuingDelay;
        volatile double loss;
        volatile double availableBandwidth;
        volatile double totalBandwidth;
        @display("i=block/network");
    
    gates:
        inout gate[];
        output slave;
        inout domain;
}

simple slave_controller {
    parameters:
        @display("i=block/control");
    
    gates:
        input in[];
        inout domain;
}

simple domain_controller {
    parameters:
        @display("i=block/app");
    
    gates:
        inout slave[];
        inout super;
        inout gate[]; // Interface with switch nodes
}

simple super_controller {
    parameters:
        @display("i=block/app");
    
    gates:
        inout domain[];
}

// =========================
//      SDN Network
// =========================

network sdn_network {
    types:
        channel Channel extends ned.DelayChannel {
            delay = 100ms;  // This value can be randomized later
        }
    
    submodules:
        switches[13]: sdn_switch;
        slave[4]: slave_controller;
        domain[2]: domain_controller;
        super: super_controller;
    
    connections:
        
        // =========================
        //      SDN Switch Topology
        // =========================
        switches[0].gate++ <--> Channel <--> switches[1].gate++;
        switches[0].gate++ <--> Channel <--> switches[2].gate++;
        switches[0].gate++ <--> Channel <--> switches[3].gate++;
        switches[3].gate++ <--> Channel <--> switches[2].gate++;
        switches[3].gate++ <--> Channel <--> switches[4].gate++;
        switches[3].gate++ <--> Channel <--> switches[5].gate++;
        switches[4].gate++ <--> Channel <--> switches[6].gate++;
        switches[5].gate++ <--> Channel <--> switches[7].gate++;
        switches[4].gate++ <--> Channel <--> switches[9].gate++;
        switches[10].gate++ <--> Channel <--> switches[7].gate++;
        switches[10].gate++ <--> Channel <--> switches[9].gate++;
        switches[10].gate++ <--> Channel <--> switches[12].gate++;
        switches[9].gate++ <--> Channel <--> switches[8].gate++;
        switches[9].gate++ <--> Channel <--> switches[11].gate++;
        switches[11].gate++ <--> Channel <--> switches[12].gate++;
        switches[1].gate++ <--> Channel <--> switches[2].gate++;
        switches[6].gate++ <--> Channel <--> switches[8].gate++;
          
        
        
        // =========================
        //     Slave Controllers
        // =========================
        for i = 0..2 {
            slave[0].in++ <-- Channel <-- switches[i].slave;
        }
        for i = 3..6 {
            slave[1].in++ <-- Channel <-- switches[i].slave;
        }
        for i = 7..9 {
            slave[2].in++ <-- Channel <-- switches[i].slave;
        }
        for i = 10..12 {
            slave[3].in++ <-- Channel <-- switches[i].slave;
        }
        
        // =========================
        //     Domain Controllers
        // =========================
        slave[0].domain <--> Channel <--> domain[0].slave++;
        slave[1].domain <--> Channel <--> domain[0].slave++;
        slave[2].domain <--> Channel <--> domain[1].slave++;
        slave[3].domain <--> Channel <--> domain[1].slave++;
        
        for i = 0..6 {
            switches[i].domain <--> Channel <--> domain[0].gate++;
        }
        for i = 7..12 {
            switches[i].domain <--> Channel <--> domain[1].gate++;
        }
        
        // =========================
        //     Super Controller
        // =========================
        for i = 0..1 {
            super.domain++ <--> Channel <--> domain[i].super;
        }
}
