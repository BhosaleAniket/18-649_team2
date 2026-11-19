"""
setup.py - one-time setup: makes the HTTPS certificate and the .env secrets.

  python setup.py --ip 192.168.1.20 --out ..     (run from server/, or via Docker, see README)

--ip is the laptop address the Pi uses (the same one the proxy setup uses).
Writes <out>/certs/server.crt + server.key and <out>/.env. Copy server.crt
and the INGEST_TOKEN line to the Pi.
"""
import argparse
import datetime
import getpass
import hashlib
import ipaddress
import os
import secrets
import socket
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID


def make_cert(out: Path, ips, names):
    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "team2-vehicle-monitor")])
    san = [x509.DNSName(n) for n in names] + [x509.IPAddress(ipaddress.ip_address(i)) for i in ips]
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (x509.CertificateBuilder()
            .subject_name(name).issuer_name(name)
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - datetime.timedelta(days=1))
            .not_valid_after(now + datetime.timedelta(days=365))
            .add_extension(x509.SubjectAlternativeName(san), critical=False)
            .add_extension(x509.BasicConstraints(ca=True, path_length=None), critical=True)
            .add_extension(x509.KeyUsage(digital_signature=True, key_cert_sign=True, content_commitment=False,
                                         key_encipherment=False, data_encipherment=False, key_agreement=False,
                                         crl_sign=False, encipher_only=False, decipher_only=False), critical=True)
            .add_extension(x509.ExtendedKeyUsage([x509.oid.ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
            .add_extension(x509.SubjectKeyIdentifier.from_public_key(key.public_key()), critical=False)
            .add_extension(x509.AuthorityKeyIdentifier.from_issuer_public_key(key.public_key()), critical=False)
            .sign(key, hashes.SHA256()))
    out.mkdir(parents=True, exist_ok=True)
    (out / "server.key").write_bytes(key.private_bytes(serialization.Encoding.PEM,
                                                       serialization.PrivateFormat.PKCS8,
                                                       serialization.NoEncryption()))
    (out / "server.crt").write_bytes(cert.public_bytes(serialization.Encoding.PEM))


def hash_pw(pw: str) -> str:
    salt = secrets.token_bytes(16)
    iters = 200_000
    h = hashlib.pbkdf2_hmac("sha256", pw.encode(), salt, iters).hex()
    return f"pbkdf2_sha256:{iters}:{salt.hex()}:{h}"  # no "$": compose would read it as a variable


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", action="append", required=True, help="laptop IP the Pi connects to (repeatable)")
    ap.add_argument("--out", default="..", help="project folder (holds docker-compose.yml)")
    ap.add_argument("--user", default="team2")
    ap.add_argument("--password", help="GUI password (asked if left out)")
    a = ap.parse_args()

    out = Path(a.out)
    make_cert(out / "certs", a.ip + ["127.0.0.1"], ["localhost", socket.gethostname()])
    pw = a.password or getpass.getpass("GUI password: ")
    token = secrets.token_hex(24)
    env = (f"GUI_USER={a.user}\n"
           f"GUI_PASS_HASH={hash_pw(pw)}\n"
           f"SESSION_SECRET={secrets.token_hex(32)}\n"
           f"INGEST_TOKEN={token}\n"
           f"REDIS_PASSWORD={secrets.token_hex(24)}\n")
    (out / ".env").write_text(env)
    print(f"Wrote {out / 'certs'} and {out / '.env'}")
    print(f"On the Pi: copy certs/server.crt, and use --token {token}")


if __name__ == "__main__":
    main()
