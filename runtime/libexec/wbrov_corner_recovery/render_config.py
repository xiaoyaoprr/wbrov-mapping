"""Create one mode-0600 authenticated corner-relocalizer configuration."""
from pathlib import Path
import argparse,hashlib,json,os,re,stat,tempfile

HEX64=re.compile(r'[0-9a-f]{64}')

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--template',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--session-token',required=True)
    parser.add_argument('--database-sha256',required=True)
    args=parser.parse_args()
    token=args.session_token
    if HEX64.fullmatch(token) is None:
        raise SystemExit('session token must be 64 lowercase hex characters')
    if HEX64.fullmatch(args.database_sha256) is None:
        raise SystemExit('database digest must be 64 lowercase hex characters')
    template=Path(args.template);target=Path(args.output)
    if target.exists() or target.is_symlink() or not target.parent.is_dir():
        raise SystemExit('unsafe or existing output path')
    config=json.loads(template.read_text(encoding='utf-8'))
    config['session_token']=token
    config['session_token_sha256']=hashlib.sha256(token.encode('ascii')).hexdigest()
    config['database_sha256']=args.database_sha256
    encoded=(json.dumps(config,ensure_ascii=False,sort_keys=True,
                        separators=(',',':'),allow_nan=False)+'\n').encode()
    fd,name=tempfile.mkstemp(prefix='.corner-config.',dir=target.parent)
    try:
        with os.fdopen(fd,'wb') as stream:
            stream.write(encoded);stream.flush();os.fsync(stream.fileno())
        os.chmod(name,0o600)
        os.replace(name,target)
        info=target.stat()
        uid_ok=(not hasattr(os,'getuid') or info.st_uid==os.getuid())
        mode_ok=(os.name=='nt' or stat.S_IMODE(info.st_mode)==0o600)
        if (not stat.S_ISREG(info.st_mode) or not uid_ok or
                info.st_nlink!=1 or not mode_ok):
            raise SystemExit('output identity or mode check failed')
    finally:
        if os.path.lexists(name):os.unlink(name)
    print(json.dumps({'path':str(target),'sha256':hashlib.sha256(encoded).hexdigest(),
                      'session_token_sha256':config['session_token_sha256'],
                      'database_sha256':config['database_sha256']}))

if __name__=='__main__':main()
