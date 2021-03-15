// Hazard detection
    // We'll need to keep an array of what each instruction is
    // We'll fill the array during each ID stage (b/c that's when we decode the instr)
    // Cases:
        // A store follows a load: forward from WB to MEM without a stall
        // All other cases of forwarding to store: forward when store is in EX stage
    // General rule: pg. 306 rule with conditions in the middle of pg. 307
    // "Hazard detection happens in ID stage" - Ziyang
        // Add a field to InstrDecoded

// Hazard response (completely depends on hazard identified)
    // Forwarding from load to store w/out stall (possible b/c we descend the stages)
        // pass the WB value to MEM
    // Forwarding to store (but not from a load)
        // update value when store is in EX
    /* General rule: Each stage from ID to WB should be checked (in every cycle)
    to see if some of its values need to be updates, or if there need to be stalls */
    /* Design suggestion: Based on general rule, we'll write logic that operates
       on the array */

// Stalling
    // When do we stall?
        // When the hazard detection tells us that we should
    // For how long do we stall?
        // Again, handled in hazard detection logic
    // Where do we stall?
        // Depends on hazard (aka handled in hazard detection logic)
    // How do we stall?
        // Idea 1: Insert 1 nop between two instructions (see pg. 282 and 283); SEEMS BAD
        // Idea 2: pg. 315, replace instr that needs to be stalled with nop for x cycles
