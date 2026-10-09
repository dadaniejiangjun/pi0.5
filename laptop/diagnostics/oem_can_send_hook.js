// Observe only: never replace functions, change arguments, or call CAN APIs.
let armed = false;
let timer = null;
let sequence = 0;
const kernel = Process.getModuleByName('kernel32.dll');
const qpc = new NativeFunction(kernel.getExportByName('QueryPerformanceCounter'), 'int', ['pointer']);
const qpf = new NativeFunction(kernel.getExportByName('QueryPerformanceFrequency'), 'int', ['pointer']);
const frequency = Memory.alloc(8);
qpf(frequency);
function ticks() {
    const value = Memory.alloc(8);
    if (!qpc(value)) throw new Error('QPC failed');
    return value.readS64().toString();
}
const module = Process.getModuleByName('candle_api.dll');
const exports = module.enumerateExports().filter(e => /^candle_frame_send(@12)?$/.test(e.name));
if (exports.length !== 1) throw new Error('expected one classic candle_frame_send export');
const listener = Interceptor.attach(exports[0].address, {
    onEnter(args) {
        this.record = null;
        if (!armed) return;
        try {
            const frame = args[2];
            const dlc = frame.add(8).readU8();
            const bytes = new Uint8Array(frame.add(12).readByteArray(8));
            this.record = {event: 'oem_send', sequence: ++sequence, source_qpc_ticks: ticks(),
                can_id_raw: frame.add(4).readU32(), dlc: dlc,
                channel: args[1].toUInt32() & 255, frame_channel: frame.add(9).readU8(),
                flags: frame.add(10).readU8(),
                data_hex: Array.from(bytes).map(b => b.toString(16).padStart(2, '0')).join('')};
        } catch (error) {
            send({event:'decode_error', error:String(error)});
        }
    },
    onLeave(result) {
        if (this.record) {
            this.record.api_return = result.toInt32();
            send(this.record);
        }
    }
});
// Count alternate send paths without guessing their frame layout/ABI.
const alternateListeners = [];
for (const m of Process.enumerateModules().filter(m => /^(candle_api|cando)\.dll$/i.test(m.name))) {
    for (const e of m.enumerateExports().filter(e => /^(candle_frame_send_fd(@12)?|cando_frame_send)$/.test(e.name))) {
        alternateListeners.push(Interceptor.attach(e.address, {
            onEnter(args) {
                this.record = armed ? {event:'alternate_send_call', source_qpc_ticks:ticks(),
                    module:m.name, export_name:e.name, frame_decoded:false} : null;
                if(this.record && e.name==='cando_frame_send') {
                    try {
                        // Local disassembly proves second argument is a 24-byte
                        // buffer passed to USB write. Retain raw bytes so layout
                        // interpretation can be checked against UI values.
                        const p=args[1];
                        const bytes=new Uint8Array(p.readByteArray(24));
                        this.record.frame_hex=Array.from(bytes).map(b=>b.toString(16).padStart(2,'0')).join('');
                        this.record.event='oem_send';
                        this.record.sequence=++sequence;
                        this.record.can_id_raw=p.add(4).readU32();
                        this.record.dlc=p.add(8).readU8();
                        this.record.frame_channel=p.add(9).readU8();
                        this.record.flags=p.add(10).readU8();
                        this.record.data_hex=Array.from(bytes.slice(12,20)).map(b=>b.toString(16).padStart(2,'0')).join('');
                        this.record.frame_decoded=true;
                        this.record.layout_status='24byte_USB_buffer_proven_CAN_offsets_require_UI_crosscheck';
                    } catch(error) {send({event:'decode_error',error:String(error)});}
                }
            },
            onLeave(result) {
                if(this.record) { this.record.api_return=result.toInt32(); send(this.record); }
            }
        }));
    }
}
rpc.exports = {
    info() { return {pid:Process.id, arch:Process.arch, module:module.path,
        export_name:exports[0].name, qpc_frequency_hz:frequency.readS64().toString(),
        can_api_called_by_logger:false, arguments_modified:false, armed:armed,
        alternate_send_hooks:alternateListeners.length}; },
    arm() {
        if (armed || timer !== null) throw new Error('one capture per attachment');
        armed=true;
        send({event:'capture_start',source_qpc_ticks:ticks()});
        timer=setTimeout(() => {
            armed=false;
            send({event:'capture_end',source_qpc_ticks:ticks()});
        },5000);
    },
    stop() { armed=false; if(timer!==null) clearTimeout(timer); listener.detach();
        for(const observer of alternateListeners) observer.detach(); }
};
