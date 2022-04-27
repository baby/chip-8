// CHIP-8 Emulator
// Based off of example from Rust in Action by Tim McNamara
// Additional instructions referenced from:
// http://www.cs.columbia.edu/~sedwards/classes/2016/4840-spring/designs/Chip8.pdf

struct CPU {
    registers: [u8; 16],
    index_register: usize,
    program_counter: usize,
    memory: [u8; 4096],
    stack: [u16; 16],
    stack_pointer: usize,
}

impl CPU {
    fn read_opcode(&self) -> u16 {
        let p = self.program_counter;
        let op_byte1 = self.memory[p] as u16;
        let op_byte2 = self.memory[p + 1] as u16;

        op_byte1 << 8 | op_byte2
    }

    fn dump_registers(&self) {
        println!("----------------------------------");
        for (idx, register) in self.registers.iter().enumerate() {
            if idx % 3 == 0 {
                println!("[{}] -> {}", idx, register);
            } else {
                print!("[{}] -> {} ", idx, register);
            }
        }
        println!("----------------------------------");
    }

    fn run(&mut self) {
        loop {
            let opcode = self.read_opcode();
            self.program_counter += 2;

            let c = ((opcode & 0xF000) >> 12) as u8;
            let x = ((opcode & 0x0F00) >> 8) as u8;
            let y = ((opcode & 0x00F0) >> 4) as u8;
            let d = ((opcode & 0x000F) >> 0) as u8;
            let kk = (opcode & 0x00FF) as u8;
            let minor = (opcode & 0x000F) as u8;
            let nnn = opcode & 0x0FFF;

            match opcode {
                0x0000 => {
                    return;
                }
                0x00EE => self.ret(),
                0x00E0 => self.clear_screen(),
                0x1000..=0x1FFF => self.jmp(nnn),
                0x2000..=0x2FFF => self.call(nnn),
                0x3000..=0x3FFF => self.se(x, kk),
                0x4000..=0x4FFF => self.sne(x, kk),
                0x5000..=0x5FFF => self.se_xy(x, y),
                0x6000..=0x6FFF => self.ld(x, kk),
                0x7000..=0x7FFF => self.add(x, kk),
                0x8000..=0x8FFF => {
                    match minor {
                        1 => { self.or_xy(x, y) },
                        2 => { self.and_xy(x, y) },
                        3 => { self.xor_xy(x, y) },
                        4 => { self.add_xy(x, y) },
                        5 => { self.sub_xy(x, y) },
                        7 => { self.subn_xy(x, y) },
                        _ => { todo!("opcode: {:04x}", opcode); }
                    }
                },
                0x9000..=0x9FFF => self.sne_xy(x, y),
                0xA000..=0xAFFF => self.ldi(nnn),
                0xB000..=0xBFFF => self.jp_v0(nnn),
                _ => todo!("opcode {:04x}", opcode),
            }
        }
    }

    // CALL: Call subroutine at address
    fn call(&mut self, addr: u16) {
        let sp = self.stack_pointer;
        let stack = &mut self.stack;

        if sp > stack.len() {
            panic!("Stack overflow!");
        }

        stack[sp] = self.program_counter as u16;
        self.stack_pointer += 1;
        self.program_counter = addr as usize;
        println!("[CALL] Addr: 0x{}", addr as usize);
    }

    // RET: Return from subroutine
    fn ret(&mut self) {
        if self.stack_pointer == 0 {
            panic!("Stack underflow");
        }

        self.stack_pointer -= 1;
        let call_addr = self.stack[self.stack_pointer];
        self.program_counter = call_addr as usize;
        println!("[RET] Call Addr: 0x{}", call_addr as usize);
    }

    // CLS: Clear the display (does nothing)
    fn clear_screen(&self) {
        println!("Screen cleared");
    }

    // JP: Jumps to address
    fn jmp(&mut self, addr: u16) {
        println!("[JMP] Jumping to {:#04x}", addr);
        self.program_counter = addr as usize;
    }

    // LD: Put value kk into register Vx
    fn ld(&mut self, vx: u8, kk: u8) {
        self.registers[vx as usize] = kk;
        println!("Called LD");
    }

    // ADD: Add value kk to value Vx and store in Vx
    fn add(&mut self, vx: u8, kk: u8) {
        let vx_ = self.registers[vx as usize];
        
        let (val, overflow) = vx_.overflowing_add(kk);
        self.registers[vx as usize] = val;

        if overflow {
            self.registers[0xF] = 1;
        } else {
            self.registers[0xF] = 0;
        }
    }

    // SE Vx, byte: Skip next instruction if Vx is equal to kk
    fn se(&mut self, vx: u8, kk: u8) {
        let vx_ = self.registers[vx as usize];
        println!("Entered SE");
        if vx_ == kk {
            self.program_counter += 2;
            println!("se skipped instruction");
        }
    }

    // SNE Vx, byte: Skip next instruction if Vx is not equal to kk
    fn sne(&mut self, vx: u8, kk: u8) {
        let vx_ = self.registers[vx as usize];
        println!("Entered SNE\n vx: {}, kk: {}", vx_, kk);
        if vx_ != kk {
            self.program_counter += 2;
            println!("sne skipped instruction");
        }
    }

    // SE Vx, Vy: Skip next instruction if Vx equals Vy
    fn se_xy(&mut self, x: u8, y: u8) {
        let x_ = self.registers[x as usize];
        let y_ = self.registers[y as usize];

        if x_ == y_ {
            self.program_counter += 2;
            println!("se_xy skipped instruction");
        }
    }

    // SNE Vx, Vy: Skip next instruction if Vx is not equal to Vy
    fn sne_xy(&mut self, x: u8, y: u8) {
        let x_ = self.registers[x as usize];
        let y_ = self.registers[y as usize];

        if x_ != y_ {
            self.program_counter += 2;
            println!("sne_xy skipped instruction");
        }
    }

    // ADD Vx, Vy: Adds Vx and Vy, sets carry bit to 1 if overflow occurs 
    fn add_xy(&mut self, x: u8, y: u8) {
        let x_ = self.registers[x as usize];
        let y_ = self.registers[y as usize];

        let (val, overflow) = x_.overflowing_add(y_);
        self.registers[x as usize] = val;

        if overflow {
            self.registers[0xF] = 1;
        } else {
            self.registers[0xF] = 0;
        }
    }

    // AND Vx, Vy: Bitwise AND on Vx and Vy
    fn and_xy(&mut self, x: u8, y: u8) {
        let x_ = self.registers[x as usize];
        let y_ = self.registers[y as usize];

        self.registers[x as usize] = x_ & y_;
    }

    // OR Vx, Vy: Bitwise OR on Vx and Vy
    fn or_xy(&mut self, x: u8, y: u8) {
        let x_ = self.registers[x as usize];
        let y_ = self.registers[y as usize];

        self.registers[x as usize] = x_ | y_;
    }

    // XOR Vx, Vy: Bitwise XOR on Vx and Vy
    fn xor_xy(&mut self, x: u8, y: u8) {
        let x_ = self.registers[x as usize];
        let y_ = self.registers[y as usize];

        self.registers[x as usize] = x_ ^ y_;
    }

    // SUB Vx, Vy: Subtracts Vy from Vx, VF set to 1 if overflow occurs
    fn sub_xy(&mut self, x: u8, y: u8) {
        let x_ = self.registers[x as usize];
        let y_ = self.registers[y as usize];

        let (val, overflow) = x_.overflowing_sub(y_);

        if overflow {
            self.registers[0xF] = 1;
        } else {
            self.registers[0xF] = 0;
        }

        self.registers[x as usize] = val;
    }

    // SUBN Vx, Vy: Subtracts Vx from Vy, VF set to 1 if overflow occurs
    fn subn_xy(&mut self, x: u8, y: u8) {
        let x_ = self.registers[x as usize];
        let y_ = self.registers[y as usize];

        let (val, overflow) = y_.overflowing_sub(x_);

        if overflow {
            self.registers[0xF] = 1;
        } else {
            self.registers[0xF] = 0;
        }

        self.registers[x as usize] = val;
    }

    // LD I: Set index register to nnn
    fn ldi(&mut self, addr: u16) {
        self.index_register = addr as usize;
        println!("Index Register: {:#04x}", self.index_register);
    }

    // JP V0: Jumps to address + the value of register 0
    fn jp_v0(&mut self, addr: u16) {
        let v = self.registers[0 as usize];
        self.program_counter = (addr + v as u16) as usize;
        println!("[JMP V0]: Jumping to {:#04x}", self.program_counter);
    }
}

fn main() {
    let mut cpu = CPU {
        registers: [0; 16],
        index_register: 0,
        memory: [0; 4096],
        program_counter: 0,
        stack: [0; 16],
        stack_pointer: 0,
    };

    cpu.registers[0] = 3; // Set register 0 to 0x3

    let mem = &mut cpu.memory;

    mem[0x000] = 0x21; mem[0x001] = 0x00; // Call function at 0x100
    mem[0x002] = 0x00; mem[0x003] = 0x00; // Halt (never reached)

    let recurse = [
        0x11, 0x02, // Jump to 0x102
        0xB0, 0xFD, // Jump to 0xFD + V0 (0x3) == 0x100
        ];

    mem[0x100..0x104].copy_from_slice(&recurse);

    println!("Starting...");

    cpu.run();

    println!("Finished executing.");
    cpu.dump_registers();
}
