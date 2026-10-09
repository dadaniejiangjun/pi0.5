import sys
sys.path.insert(0,'cache/tooldeps/oem_capture')
import frida
session=frida.attach(54444)
script=session.create_script('''rpc.exports={code(){
const m=Process.getModuleByName('cando.dll');
const e=m.enumerateExports().find(e=>e.name==='cando_frame_send');
let p=e.address;const instructions=[];
for(let n=0;n<75;n++) { const i=Instruction.parse(p);instructions.push(i.toString());p=i.next;if(i.mnemonic==='ret')break; }
return instructions;
}};''')
try:
    script.load()
    print('\n'.join(script.exports_sync.code()))
finally:
    script.unload();session.detach()
