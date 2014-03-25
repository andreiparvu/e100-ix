#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>

#include <linux/slab.h>

#include <linux/pci.h>
#include <linux/etherdevice.h>

#include <linux/interrupt.h>

#include <asm/uaccess.h>

MODULE_DESCRIPTION("E100 driver");
MODULE_AUTHOR("Andrei Parvu");
MODULE_LICENSE("GPL");

#define LOG_LEVEL KERN_ALERT

#define MAX_CBS 32

#define CMD_BASE 0x02
#define SCB_POINTER_BASE 0x04

#define CUC_START (0b0001 << 4) | 0xbc00
#define CUC_RESUME (0b0010 << 4) | 0xbc00
#define CUC_LOAD_BASE (0b0110 << 4)

#define RUC_START 0b0001
#define RUC_RESUME 0b0010
#define RUC_LOAD_BASE 0b0110

#define TRUE 1
#define FALSE 0

struct e100_cb {
	__le16 status;
	__le16 cmd;
	__le32 link;

	u32 tbd_array;
	u16 tcb_byte_count;
	u8 threshold;
	u8 tbd_count;

	uint8_t data[1518];

	dma_addr_t dma_addr;
	struct e100_cb *next, *prev;
};

struct e100_data {
	struct e100_cb *tx_cbl[MAX_CBS];
	struct e100_cb *rx_cbl[MAX_CBS];

	struct pci_dev *pdev;
	struct net_device *net_dev;

	struct e100_cb *cur_tx_cb;
	struct e100_cb *cur_rx_cb;

	int has_started;

};

void set_cmd(unsigned long base, unsigned short value, int no_wait) {
	int i;

	for (i = 0; i < 20000; i++) {
		unsigned char v = inw(base);

		// Astept sa se reseteze registrul - am inteles ca asa face el
		// ack la o comanda :)
		if (!v || no_wait) {
			//printk(LOG_LEVEL "cmd succes %d %d\n", i, value);
			//mdelay(3000);

			outw(value, base);

			break;
		}

		if (i > 20)
			udelay(5);
	}

	if (i == 20000)
		printk(LOG_LEVEL "Error!!!!\n");
}

irqreturn_t e100_irq(int irq_no, void *dev_id)
{
	struct e100_data *data = (struct e100_data*)dev_id;
	struct sk_buff *skb;

	printk(LOG_LEVEL "Nasol %d %d\n", data->cur_rx_cb->status,
			data->cur_rx_cb->tcb_byte_count);

	skb = dev_alloc_skb(data->cur_rx_cb->tcb_byte_count + 2);

	memcpy(skb_put(skb, data->cur_rx_cb->tcb_byte_count),
			data->cur_rx_cb->data, data->cur_rx_cb->tcb_byte_count);

	skb->dev = data->net_dev;
	skb->protocol = eth_type_trans(skb, data->net_dev);
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	netif_rx(skb);

	data->cur_rx_cb = data->cur_rx_cb->next;

	set_cmd(data->net_dev->base_addr, 0xFF00, TRUE);
	set_cmd(data->net_dev->base_addr + CMD_BASE, RUC_RESUME, FALSE);

	return IRQ_HANDLED;
}

int e100_open(struct net_device *net_dev)
{
	struct e100_data *data = netdev_priv(net_dev);
	int ret;

	printk(LOG_LEVEL "in open");
	net_dev->base_addr = data->pdev->resource[1].start;

	outw(0, net_dev->base_addr + CMD_BASE);

	// Load address of first CB
	data->cur_tx_cb = data->tx_cbl[0];
	data->cur_rx_cb = data->rx_cbl[0];
	//printk(LOG_LEVEL "%lu\n", data->cur_cb->dma_addr);

	outl(cpu_to_le32(data->cur_rx_cb->dma_addr), net_dev->base_addr +
			SCB_POINTER_BASE);

	set_cmd(net_dev->base_addr + CMD_BASE, RUC_START, FALSE);

	data->has_started = 0;
	ret = request_irq(net_dev->irq, e100_irq, IRQF_SHARED, "e100", data);

	printk(LOG_LEVEL "%d %d\n", ret, net_dev->irq);

	return 0;
}

netdev_tx_t e100_start_xmit(struct sk_buff *skb, struct net_device *net_dev)
{
	struct e100_data *data = netdev_priv(net_dev);

	printk(LOG_LEVEL "sk buff de dimensiune %d\n", skb->len);

	memcpy(data->cur_tx_cb->data, skb->data, skb->len);
	data->cur_tx_cb->tcb_byte_count = skb->len;

	data->cur_tx_cb->prev->cmd &= ~(1 << 14);
	data->cur_tx_cb->cmd = cpu_to_le16(0x4004); // bitul S setat si bitul si
						// cmd cu valoarea 0b100
	wmb();

	if (data->has_started == 0) {
		data->has_started = 1;
		printk(LOG_LEVEL "Has started\n");
		outl(cpu_to_le32(data->cur_tx_cb->dma_addr), net_dev->base_addr +
			SCB_POINTER_BASE);
		set_cmd(net_dev->base_addr + CMD_BASE, CUC_START, FALSE);
	} else {
		set_cmd(net_dev->base_addr + CMD_BASE, CUC_RESUME, FALSE);
	}

	data->cur_tx_cb = data->cur_tx_cb->next;
	return NETDEV_TX_OK;
}

int e100_stop(struct net_device *net_dev)
{
	struct e100_data *data = netdev_priv(net_dev);

	free_irq(net_dev->irq, data);

	return 0;
}

static int e100_set_mac_address(struct net_device *netdev, void *p)
{
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

	return 0;
}

struct net_device_ops e100_ops = {
	.ndo_open = e100_open,
	.ndo_stop = e100_stop,
	.ndo_start_xmit = e100_start_xmit,
	.ndo_set_mac_address = e100_set_mac_address,
};

static void alloc_cbl(struct e100_cb *cbl[MAX_CBS], struct pci_dev *dev) {
	int i;

	for (i = 0; i < MAX_CBS; i++) {
		dma_addr_t dma_addr;

		cbl[i] = pci_alloc_consistent(dev,
				sizeof(struct e100_cb), &dma_addr);
		cbl[i]->dma_addr = dma_addr;
		cbl[i]->tbd_array = 0xffffffff;
		cbl[i]->threshold = 0xe0;
		cbl[i]->tbd_count = 0;
		cbl[i]->cmd = 0;

		if (i > 0) {
			cbl[i - 1]->link = cpu_to_le32(dma_addr);
			cbl[i - 1]->next = cbl[i];
			cbl[i]->prev = cbl[i - 1];
		}
		if (i == MAX_CBS - 1) {
			cbl[i]->link =
				cpu_to_le32(cbl[0]->dma_addr);
			cbl[i]->next = cbl[0];
			cbl[0]->prev = cbl[i];
		}
	}
}

static void free_cbl(struct e100_cb *cbl[MAX_CBS], struct pci_dev *dev) {
	int i;

	for (i = 0; i < MAX_CBS; i++) {
		pci_free_consistent(dev, sizeof(struct e100_cb), cbl[i],
				cbl[i]->dma_addr);
	}
}

	
int e100_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret;
	struct net_device *net_dev = alloc_etherdev(sizeof(struct e100_data));
	struct e100_data *data = netdev_priv(net_dev);

	ret = pci_enable_device(dev);

	if (ret < 0) {
		return -1;
	}

	pci_set_master(dev);

	data->pdev = dev;
	data->net_dev = net_dev;

	strcpy(net_dev->name, "ixeth%d");
	net_dev->netdev_ops = &e100_ops;
	ret = register_netdev(net_dev);

	net_dev->irq = dev->irq;

	alloc_cbl(data->tx_cbl, dev);
	alloc_cbl(data->rx_cbl, dev);

	printk(LOG_LEVEL "done\n");

	return 0;
}

void e100_remove(struct pci_dev *dev)
{
	struct net_device *net_dev = pci_get_drvdata(dev);
	struct e100_data *data = netdev_priv(net_dev);

	free_cbl(data->tx_cbl, dev);
	free_cbl(data->rx_cbl, dev);
}

struct pci_device_id e100_id_table[] = {
	{
		.vendor = 0x8086,
		.device = 0x1209,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID
	},
	{0,}
};

struct pci_driver e100_driver = {
	.name = "e100-ix",
	.id_table = e100_id_table,
	.probe = e100_probe,
	.remove = e100_remove
};

static int e100_init(void)
{
	int ret = pci_register_driver(&e100_driver);

	printk(LOG_LEVEL "%d\n", ret);

	return 0;
}

static void e100_exit(void)
{
	pci_unregister_driver(&e100_driver);
}

module_init(e100_init);
module_exit(e100_exit);
